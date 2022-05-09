#include "modular_slam/realsense_camera.hpp"
#include <QApplication>
#include <QMainWindow>

// #include <librealsense2/h/rs_sensor.h>
// #include <librealsense2/rs.hpp>

#include <cstdint>
#include <iostream>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include <limits>
#include <opencv2/core/hal/interface.h>
#include <opencv2/imgproc.hpp>

int main(int argc, char* argv[])
{
    mslam::RealSenseCamera rsCamera;

    rsCamera.init();

    while(true)
    {
        rsCamera.fetch();

        auto rgbd = rsCamera.recentData();

        cv::Mat rgbFrame{720, 1280, CV_8UC3, rgbd->rgbData.data()};
        cv::Mat depthFrame{720, 1280, CV_16UC1, rgbd->depthData.data()};
        cv::Mat img_color;
        cv::convertScaleAbs(depthFrame, img_color, 255.0 / 4000);
        applyColorMap(img_color, img_color, cv::COLORMAP_HOT);

        cv::addWeighted(img_color, 0.7, rgbFrame, 0.3, 0, img_color);

        cv::imshow("RGB", img_color);
        cv::waitKey(30);
    }
}
