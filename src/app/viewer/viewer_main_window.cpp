#include "viewer_main_window.hpp"
#include <qmainwindow.h>

namespace mslam
{

ViewerMainWindow::ViewerMainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::ViewerMainWindow)
{
    ui->setupUi(this);
    connect(ui->pauseResumeAction, &QAction::triggered, this, &ViewerMainWindow::onPauseResume);
}

void ViewerMainWindow::onPauseResume()
{
    isPaused = !isPaused;

    if(isPaused)
        emit paused();
    else
        emit resumed();
}

} // namespace mslam
