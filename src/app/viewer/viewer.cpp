#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QMainWindow>

#include <cstdint>
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
#include "modular_slam/orb_feature.hpp"
#include "modular_slam/realsense_camera.hpp"
#include "modular_slam/rgbd_file_provider.hpp"
#include "modular_slam/rgbd_frame.hpp"
#include "modular_slam/slam3d_types.hpp"
#include "modular_slam/slam_builder.hpp"

#include "viewer_main_window.hpp"

#include "modular_slam/modular_slam.hpp"
#include "slam_thread.hpp"

#include <QPixmap>
#include <memory>
#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>

struct ViewerArgs
{
    std::optional<std::string> tumFile;
    bool useRealSense;
};

ViewerArgs parseArgs(QApplication& app)
{
    ViewerArgs args;
    QCommandLineParser parser;
    parser.setApplicationDescription("Runs Modular SLAM with Viewer");

    parser.addOption({{"t", "tum_file"}, "Provides data from TUM output of associate.py script", "file"});
    parser.addOption({{"r", "realsense"}, "Provides data from RealSense device"});

    parser.addHelpOption();
    parser.addVersionOption();
    parser.process(app);

    args.useRealSense = parser.isSet("realsense");
    args.tumFile = parser.isSet("tum_file") ? std::make_optional(parser.value("tum_file").toStdString()) : std::nullopt;

    return args;
}

void handleArgs(const ViewerArgs& /*args*/) {}

auto buildSlam(const ViewerArgs& args)
{
    mslam::SlamBuilder<mslam::RgbdFrame, mslam::slam3d::SensorState, mslam::Vector3> slamBuilder;

    std::shared_ptr<mslam::DataProviderInterface<mslam::RgbdFrame>> dataProvider =
        std::make_shared<mslam::RealSenseCamera>();

    if(args.tumFile.has_value())
    {
        const auto rgbdPaths = mslam::readTumRgbdDataset(args.tumFile.value());
        dataProvider = std::make_shared<mslam::RgbdFileProvider>(rgbdPaths, mslam::tumRgbdCameraParams());
    }

    auto frontend = std::make_shared<mslam::RgbdFeatureFrontend>(
        std::make_shared<mslam::OpenCvRansacPnp>(), std::make_shared<mslam::OrbOpenCvDetector>(),
        std::make_shared<mslam::OrbOpenCvMatcher>(), std::make_shared<mslam::BasicFeatureMapComponentsFactory>());

    slamBuilder.addParameterHandler(std::make_shared<mslam::BasicParameterHandler>())
        .addDataProvider(dataProvider)
        .addFrontend(frontend)
        .addBackend(std::make_shared<mslam::CeresBackend>())
        .addMap(std::make_shared<mslam::BasicMap>());

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

    auto mainWindow = new mslam::ViewerMainWindow();
    SlamThread* slamThread = new SlamThread(mainWindow);
    slamThread->setSlam(std::move(slam));
    slamThread->start();

    QObject::connect(slamThread, &SlamThread::newRgbImageAvailable, mainWindow, &mslam::ViewerMainWindow::setImage);
    QObject::connect(slamThread, &SlamThread::newDepthImageAvailable, mainWindow,
                     &mslam::ViewerMainWindow::setDepthImage);
    QObject::connect(slamThread, &SlamThread::newPointsAvailable, mainWindow, &mslam::ViewerMainWindow::setPoints);
    QObject::connect(slamThread, &SlamThread::newSlamStatisticsAvailable, mainWindow,
                     &mslam::ViewerMainWindow::setSlamStatistics);
    QObject::connect(slamThread, &SlamThread::keyframeAdded, mainWindow, &mslam::ViewerMainWindow::addKeyframe);
    QObject::connect(slamThread, &SlamThread::currentFrameChanged, mainWindow,
                     &mslam::ViewerMainWindow::setCurrentFrame);
    QObject::connect(mainWindow, &mslam::ViewerMainWindow::paused, slamThread, &SlamThread::pause);
    QObject::connect(mainWindow, &mslam::ViewerMainWindow::resumed, slamThread, &SlamThread::resume);

    mainWindow->show();

    int result = app.exec();

    slamThread->stop();
    slamThread->wait();

    return result;
}
