#include "viewer_main_window.hpp"
#include "pointcloud_viewer.hpp"

#include <QSettings>
#include <qmainwindow.h>

namespace mslam
{

ViewerMainWindow::ViewerMainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::ViewerMainWindow)
{
    ui->setupUi(this);
    connect(ui->pauseResumeAction, &QAction::triggered, this, &ViewerMainWindow::onPauseResume);
    connect(ui->actionResetCamera, &QAction::triggered, ui->pointcloudViewer, &PointcloudViewer::resetCamera);
    loadSettings();
}

void ViewerMainWindow::onPauseResume()
{
    isPaused = !isPaused;

    if(isPaused)
        emit paused();
    else
        emit resumed();
}

void ViewerMainWindow::saveSettings()
{
    QSettings settings("mslam", "Viewer");
    settings.setValue("windowState", saveState());
}

void ViewerMainWindow::loadSettings()
{
    QSettings settings("mslam", "Viewer");
    restoreState(settings.value("windowState").toByteArray());
}

void ViewerMainWindow::closeEvent(QCloseEvent* event)
{
    saveSettings();
    emit isClosing();
    QMainWindow::closeEvent(event);
}

} // namespace mslam
