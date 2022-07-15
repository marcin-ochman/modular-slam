#ifndef VIEWER_MAIN_WINDOW_HPP_
#define VIEWER_MAIN_WINDOW_HPP_

#include "ui_viewer_main_window.h"
#include <QMainWindow>

namespace Ui
{
class ViewerMainWindow;
}

namespace mslam
{
class ViewerMainWindow : public QMainWindow
{
  public:
    ViewerMainWindow(QWidget* parent = nullptr);

  public slots:
    void setImage(const QPixmap& pixmap) { ui->imageViewer->drawImage(pixmap); }

  private:
    std::unique_ptr<Ui::ViewerMainWindow> ui;
};
} // namespace mslam

#endif // VIEWER_MAIN_WINDOW_HPP_
