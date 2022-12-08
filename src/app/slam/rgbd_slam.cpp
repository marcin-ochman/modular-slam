#include "modular_slam/basic_parameters_handler.hpp"
#include "modular_slam/basic_types.hpp"
#include "modular_slam/ceres_backend.hpp"
#include "modular_slam/data_provider.hpp"
#include "modular_slam/frontend/rgbd_feature_frontend.hpp"
#include "modular_slam/min_mse_tracker.hpp"
#include "modular_slam/orb_feature.hpp"
#include "modular_slam/realsense_camera.hpp"
#include "modular_slam/rgbd_file_provider.hpp"
#include "modular_slam/rgbd_frame.hpp"
#include "modular_slam/slam3d_types.hpp"
#include "modular_slam/slam_builder.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <memory>
#include <opencv2/highgui.hpp>

#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <optional>
#include <ostream>

struct RgbdSlamProgramArgs
{
    std::optional<std::string> tumFile;
    bool useRealSense;
};

RgbdSlamProgramArgs parseArgs(QApplication& app)
{
    RgbdSlamProgramArgs args;
    QCommandLineParser parser;
    parser.setApplicationDescription("Runs RGBD Feature-based Modular SLAM with Viewer");
    parser.addOption({{"t", "tum_file"}, "Provides data from TUM output of associate.py script", "file"});
    parser.addOption({{"r", "realsense"}, "Provides data from RealSense device"});
    parser.addHelpOption();
    parser.addVersionOption();
    parser.process(app);

    args.useRealSense = parser.isSet("realsense");
    args.tumFile = parser.isSet("tum_file") ? std::make_optional(parser.value("tum_file").toStdString()) : std::nullopt;

    return args;
}

int main(int argc, char* argv[])
{
    QApplication::setApplicationName("Feature-based RGBD Modular SLAM");
    QApplication::setApplicationVersion("1.0");
    QApplication app{argc, argv};

    auto args = parseArgs(app);

    mslam::SlamBuilder<mslam::RgbdFrame, mslam::slam3d::SensorState, mslam::Vector3> slamBuilder;

    std::shared_ptr<mslam::DataProviderInterface<mslam::RgbdFrame>> dataProvider =
        std::make_shared<mslam::RealSenseCamera>();
    if(args.tumFile.has_value())
    {
        const auto rgbdPaths = mslam::readTumRgbdDataset(args.tumFile.value());
        dataProvider = std::make_shared<mslam::RgbdFileProvider>(rgbdPaths, mslam::tumRgbdCameraParams());
    }

    auto frontend = std::make_shared<mslam::RgbdFeatureFrontend>(std::make_shared<mslam::MinMseTracker>(),
                                                                 std::make_shared<mslam::OrbFeatureDetector>());

    slamBuilder.addParameterHandler(std::make_shared<mslam::BasicParameterHandler>())
        .addDataProvider(dataProvider)
        .addFrontend(frontend)
        .addBackend(std::make_shared<mslam::CeresBackend>());

    auto slam = slamBuilder.build();
    slam->init();

    while(true)
    {
        slam->process();
        auto data = dataProvider->recentData();

        cv::Mat mat{data->rgb.size.height, data->rgb.size.width, CV_8UC3, data->rgb.data.data()};
        cv::imshow("rgb", mat);
        cv::waitKey(1);
    }

    return app.exec();
}
