#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QMainWindow>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>

#include "modular_slam/basic_feature_map_components_factory.hpp"
#include "modular_slam/basic_map.hpp"
#include "modular_slam/basic_parameters_handler.hpp"
#include "modular_slam/basic_types.hpp"
#include "modular_slam/ceres_backend.hpp"
#include "modular_slam/ceres_reprojection_error_pnp.hpp"
#include "modular_slam/cv_ransac_pnp.hpp"
#include "modular_slam/data_provider.hpp"
#include "modular_slam/frontend/rgbd_feature_frontend.hpp"
#include "modular_slam/frontend_output.hpp"
#include "modular_slam/orb_feature.hpp"
#include "modular_slam/realsense_camera.hpp"
#include "modular_slam/rgbd_file_provider.hpp"
#include "modular_slam/rgbd_frame.hpp"
#include "modular_slam/rgbd_slam_types.hpp"
#include "modular_slam/slam3d_types.hpp"
#include "modular_slam/slam_builder.hpp"

#include "viewer_main_window.hpp"

#include "modular_slam/modular_slam.hpp"
#include "slam_thread.hpp"

#include <QPixmap>
#include <memory>
#include <optional>
#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>
#include <string_view>

enum class TrajectoryFileFormat
{
    KITTI,
    TUM
};

struct TrajectoryFileArgs
{
    std::string trajectoryFilePath;
    TrajectoryFileFormat format;
};

struct ViewerArgs
{
    std::optional<std::string> tumFile;
    std::optional<TrajectoryFileArgs> output;
    bool useRealSense;
};

std::optional<TrajectoryFileArgs> makeTrajectoryFileArgs(const std::string type, const std::string& path)
{
    const std::unordered_map<std::string, TrajectoryFileFormat> toEnum = {{"kitti", TrajectoryFileFormat::KITTI},
                                                                          {"tum", TrajectoryFileFormat::TUM}};

    auto foundIt = toEnum.find(type);

    if(path.empty() || foundIt == std::end(toEnum))
        return std::nullopt;

    TrajectoryFileArgs fileArgs;
    fileArgs.format = foundIt->second;
    fileArgs.trajectoryFilePath = path;

    return fileArgs;
}

ViewerArgs parseArgs(QApplication& app)
{
    ViewerArgs args;
    QCommandLineParser parser;
    parser.setApplicationDescription("Runs Modular SLAM with Viewer");

    parser.addOption({{"t", "tum_file"}, "Provides data from TUM output of associate.py script", "file"});
    parser.addOption({{"r", "realsense"}, "Provides data from RealSense device"});
    parser.addOption({{"o", "trajectory_output"}, "Path to output trajectory", "output"});
    parser.addOption({{"f", "trajectory_format"}, "Trajectory format [KITTI, TUM]", "format"});

    parser.addHelpOption();
    parser.addVersionOption();
    parser.process(app);

    args.useRealSense = parser.isSet("realsense");
    args.tumFile = parser.isSet("tum_file") ? std::make_optional(parser.value("tum_file").toStdString()) : std::nullopt;
    args.output = makeTrajectoryFileArgs(
        parser.isSet("trajectory_format") ? parser.value("trajectory_format").toLower().toStdString() : "",
        parser.isSet("trajectory_output") ? parser.value("trajectory_output").toStdString() : "");

    return args;
}

void handleArgs(const ViewerArgs& /*args*/) {}

class KittiLocalizationDumper
{
  public:
    explicit KittiLocalizationDumper(const std::string& path) : output(path) {}

    template <typename SensorStateType, typename LandmarkStateType, typename ObservationType>
    void operator()(const mslam::FrontendOutput<SensorStateType, LandmarkStateType, ObservationType>& frontendOutput);

  private:
    std::ofstream output;
};

template <typename SensorStateType, typename LandmarkStateType, typename ObservationType>
void KittiLocalizationDumper::operator()(
    const mslam::FrontendOutput<SensorStateType, LandmarkStateType, ObservationType>& frontendOutput)
{
    constexpr auto rows = 3;
    constexpr auto cols = 4;

    Eigen::Matrix<double, rows, cols> pose;
    pose.block<3, 3>(0, 0) = frontendOutput.pose.orientation.toRotationMatrix();
    pose.block<3, 1>(0, 3) = frontendOutput.pose.position;

    for(auto i = 0; i < rows; i++)
    {
        for(auto j = 0; j < cols; j++)
            output << pose(i, j) << " ";
    }

    output << "\n";
}

class TumLocalizationDumper
{
  public:
    explicit TumLocalizationDumper(const std::string& path) : output(path)
    {
        output.setf(std::ios::fixed);
        output.precision(6);
    }

    template <typename SensorStateType, typename LandmarkStateType, typename ObservationType>
    void operator()(const mslam::FrontendOutput<SensorStateType, LandmarkStateType, ObservationType>& frontendOutput);

  private:
    std::ofstream output;
};

template <typename SensorStateType, typename LandmarkStateType, typename ObservationType>
void TumLocalizationDumper::operator()(
    const mslam::FrontendOutput<SensorStateType, LandmarkStateType, ObservationType>& frontendOutput)
{
    const auto& position = frontendOutput.pose.position;
    const auto& quaternion = frontendOutput.pose.orientation;

    output << frontendOutput.timestamp.timePoint << " ";
    output << position.x() << " " << position.y() << " " << position.z() << " ";
    output << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << " " << quaternion.w();
    output << "\n";
}

auto buildSlam(const ViewerArgs& args)
{
    mslam::SlamBuilder<mslam::RgbdFrame, mslam::slam3d::SensorState, mslam::Vector3, mslam::rgbd::RgbdKeypoint>
        slamBuilder;

    std::shared_ptr<mslam::DataProviderInterface<mslam::RgbdFrame>> dataProvider =
        std::make_shared<mslam::RealSenseCamera>();

    if(args.tumFile.has_value())
    {
        const auto rgbdPaths = mslam::readTumRgbdDataset(args.tumFile.value());
        dataProvider = std::make_shared<mslam::RgbdFileProvider>(rgbdPaths, mslam::tumRgbdCameraParams());
    }

    auto map = std::make_shared<mslam::BasicMap>();
    auto frontend = std::make_shared<mslam::RgbdFeatureFrontend>(
        std::make_shared<mslam::OpenCvRansacPnp>(), std::make_shared<mslam::OrbOpenCvDetector>(),
        std::make_shared<mslam::OrbOpenCvMatcher>(), std::make_shared<mslam::BasicFeatureMapComponentsFactory>(), map);
    auto backend = std::make_shared<mslam::CeresBackend>();

    backend->setMap(map);

    slamBuilder.addParameterHandler(std::make_shared<mslam::BasicParameterHandler>())
        .addDataProvider(dataProvider)
        .addFrontend(frontend)
        .addBackend(backend)
        .addMap(map)
        .registerDataFetchedAction([backend](std::shared_ptr<mslam::RgbdFrame> frame)
                                   { backend->setCameraParameters(frame->depth.cameraParameters); });

    if(args.output.has_value())
    {
        if(args.output->format == TrajectoryFileFormat::KITTI)
        {
            auto dumper = std::make_shared<KittiLocalizationDumper>(args.output->trajectoryFilePath);
            slamBuilder.registerFrontendFinishedAction(
                [dumper](const auto& frontendOutput)
                {
                    auto& dumperRef = *dumper;
                    dumperRef(frontendOutput);
                });
        }
        else
        {
            auto dumper = std::make_shared<TumLocalizationDumper>(args.output->trajectoryFilePath);
            slamBuilder.registerFrontendFinishedAction(
                [dumper](const auto& frontendOutput)
                {
                    auto& dumperRef = *dumper;
                    dumperRef(frontendOutput);
                });
        }
    }

    return slamBuilder.build();
}

int main(int argc, char* argv[])
{
    spdlog::cfg::load_env_levels();

    QApplication::setApplicationName("Modular SLAM Viewer");
    QApplication::setApplicationVersion("1.0");

    QApplication app{argc, argv};
    auto args = parseArgs(app);
    handleArgs(args);

    auto slam = buildSlam(args);

    mslam::ViewerMainWindow mainWindow;
    SlamThread* slamThread = new SlamThread(&mainWindow);
    slamThread->setSlam(std::move(slam));
    slamThread->start();

    QObject::connect(slamThread, &SlamThread::rgbImageChanged, &mainWindow, &mslam::ViewerMainWindow::setImage);
    QObject::connect(slamThread, &SlamThread::depthImageChanged, &mainWindow, &mslam::ViewerMainWindow::setDepthImage);
    QObject::connect(slamThread, &SlamThread::cameraPointsChanged, &mainWindow,
                     &mslam::ViewerMainWindow::setCurrentCameraPoints);
    QObject::connect(slamThread, &SlamThread::slamStatisticsChanged, &mainWindow,
                     &mslam::ViewerMainWindow::setSlamStatistics);
    QObject::connect(slamThread, &SlamThread::keyframeAdded, &mainWindow, &mslam::ViewerMainWindow::addKeyframe);
    QObject::connect(slamThread, &SlamThread::currentFrameChanged, &mainWindow,
                     &mslam::ViewerMainWindow::setCurrentFrame);
    QObject::connect(slamThread, &SlamThread::landmarkPointsChanged, &mainWindow,
                     &mslam::ViewerMainWindow::setLandmarkPoints);
    QObject::connect(&mainWindow, &mslam::ViewerMainWindow::paused, slamThread, &SlamThread::pause);
    QObject::connect(&mainWindow, &mslam::ViewerMainWindow::resumed, slamThread, &SlamThread::resume);
    QObject::connect(&mainWindow, &mslam::ViewerMainWindow::isClosing, slamThread, &SlamThread::requestInterruption);

    mainWindow.show();

    int result = app.exec();

    slamThread->stop();
    slamThread->wait();

    return result;
}
