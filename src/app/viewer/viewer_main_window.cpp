#include "viewer_main_window.hpp"
#include <qmainwindow.h>

namespace mslam
{

ViewerMainWindow::ViewerMainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::ViewerMainWindow)
{
    ui->setupUi(this);
}

} // namespace mslam
