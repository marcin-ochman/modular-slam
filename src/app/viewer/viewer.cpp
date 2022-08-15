#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QMainWindow>

#include <cstdint>
#include <iostream>
#include <limits>

#include "modular_slam/realsense_camera.hpp"
#include "viewer_main_window.hpp"

#include "modular_slam/modular_slam.hpp"
#include "slam_thread.hpp"
#include <QThread>
#include <qobject.h>
#include <qpixmap.h>

struct ViewerArgs
{
};

ViewerArgs parseArgs(QApplication& app)
{
    ViewerArgs args;
    QCommandLineParser parser;
    parser.setApplicationDescription("Runs Modular SLAM with Viewer");
    parser.addHelpOption();
    parser.addVersionOption();
    parser.process(app);

    // TODO: fill args
    //
    return args;
}

int main(int argc, char* argv[])
{
    QApplication::setApplicationName("Modular SLAM Viewer");
    QApplication::setApplicationVersion("1.0");

    QApplication app{argc, argv};
    auto args = parseArgs(app);

    auto mainWindow = new mslam::ViewerMainWindow{};
    SlamThread* slamThread = new SlamThread(mainWindow);
    slamThread->start();

    QObject::connect(slamThread, &SlamThread::newRgbImageAvailable, mainWindow, &mslam::ViewerMainWindow::setImage);
    QObject::connect(slamThread, &SlamThread::newDepthImageAvailable, mainWindow,
                     &mslam::ViewerMainWindow::setDepthImage);
    QObject::connect(slamThread, &SlamThread::newPointsAvailable, mainWindow, &mslam::ViewerMainWindow::setPoints);

    mainWindow->show();

    int result = app.exec();

    slamThread->stop();
    slamThread->wait();

    return result;
}
