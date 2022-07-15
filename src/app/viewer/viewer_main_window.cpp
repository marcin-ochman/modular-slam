#include "viewer_main_window.hpp"
#include <qmainwindow.h>

namespace mslam
{

ViewerMainWindow::ViewerMainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::ViewerMainWindow)
{
    ui->setupUi(this);
    ui->imageViewer->drawImage(QPixmap("/home/marcin/Downloads/MB C180/20190217_163616.jpg"));
}

} // namespace mslam
