#include "viewer_main_window.hpp"
#include "pointcloud_viewer.hpp"

#include <QFileDialog>
#include <QSettings>
#include <qfiledialog.h>

namespace mslam
{

ViewerMainWindow::ViewerMainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::ViewerMainWindow)
{
    ui->setupUi(this);
    connect(ui->pauseResumeAction, &QAction::triggered, this, &ViewerMainWindow::onPauseResume);
    connect(ui->actionResetCamera, &QAction::triggered, ui->pointcloudViewer, &PointcloudViewer::resetCamera);

    connect(ui->saveRgbAction, &QAction::triggered, this, &ViewerMainWindow::saveRgbImage);
    connect(ui->save3dViewAction, &QAction::triggered, this, &ViewerMainWindow::save3dView);

    loadSettings();
}

void ViewerMainWindow::onPauseResume()
{
    isPaused = !isPaused;
    ui->saveRgbAction->setEnabled(isPaused);
    ui->save3dViewAction->setEnabled(isPaused);

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

void ViewerMainWindow::save3dView()
{
    auto path = QFileDialog::getSaveFileName(this);

    if(path.isEmpty())
        return;

    auto img = ui->pointcloudViewer->grabFramebuffer();
    img.save(path);
}

void ViewerMainWindow::saveRgbImage()
{

    auto path = QFileDialog::getSaveFileName(this);

    if(path.isEmpty())
        return;

    auto img = ui->imageViewer->grabImage();
    img.save(path);
}

} // namespace mslam
