#include <QApplication>
#include <QMainWindow>

#include <cstdint>
#include <iostream>
#include <limits>

#include "modular_slam/realsense_camera.hpp"
#include "viewer_main_window.hpp"

int main(int argc, char* argv[])
{
    QApplication app{argc, argv};

    auto mainWindow = new mslam::ViewerMainWindow{};
    mainWindow->show();

    return app.exec();
}
