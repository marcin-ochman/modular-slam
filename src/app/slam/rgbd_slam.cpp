#include "modular_slam/backend/ceres_backend.hpp"
#include "modular_slam/backend/ceres_reprojection_error_pnp.hpp"
#include "modular_slam/cv_ransac_pnp.hpp"
#include "modular_slam/frontend/rgbd_feature_frontend.hpp"
#include "modular_slam/map/basic_feature_map_components_factory.hpp"
#include "modular_slam/map/basic_map.hpp"
#include "modular_slam/orb_feature.hpp"
#include "modular_slam/parameters/basic_parameters_handler.hpp"
#include "modular_slam/sensors/realsense_camera.hpp"
#include "modular_slam/sensors/rgbd_file_provider.hpp"
#include "modular_slam/slam_builder.hpp"
#include "modular_slam/types/rgbd_slam_types.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <memory>

#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <optional>
#include <ostream>
#include <spdlog/cfg/env.h>
#include <spdlog/spdlog.h>

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
    spdlog::cfg::load_env_levels();

    QApplication::setApplicationName("Feature-based RGBD Modular SLAM");
    QApplication::setApplicationVersion("1.0");
    QApplication app{argc, argv};

    auto args = parseArgs(app);

    mslam::SlamBuilder<mslam::RgbdFrame, mslam::slam3d::SensorState, mslam::Vector3,
                       mslam::rgbd::RgbdOrbKeypointDescriptor>
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

    slamBuilder.addParameterHandler(std::make_shared<mslam::BasicParameterHandler>())
        .addDataProvider(dataProvider)
        .addFrontend(frontend)
        .addBackend(std::make_shared<mslam::CeresBackend>())
        .addMap(map);

    auto slam = slamBuilder.build();
    slam->init();

    while(true)
    {
        slam->process();
        auto data = dataProvider->recentData();
    }

    return app.exec();
}
