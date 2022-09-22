#include "modular_slam/basic_parameters_handler.hpp"
#include "modular_slam/basic_types.hpp"
#include "modular_slam/ceres_backend.hpp"
#include "modular_slam/frontend/rgbd_feature_frontend.hpp"
#include "modular_slam/min_mse_tracker.hpp"
#include "modular_slam/orb_feature.hpp"
#include "modular_slam/realsense_camera.hpp"
#include "modular_slam/rgbd_frame.hpp"
#include "modular_slam/slam3d_types.hpp"
#include "modular_slam/slam_builder.hpp"

#include <iostream>
#include <memory>
#include <opencv2/highgui.hpp>

int main(int argc, char* argv[])
{
    mslam::SlamBuilder<mslam::RgbdFrame, mslam::slam3d::SensorState, mslam::Vector3> slamBuilder;

    auto dataProvider = std::make_shared<mslam::RealSenseCamera>();
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

    // .addMap(std::make_shared<>());

    return 0;
}
