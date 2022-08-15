#ifndef MSLAM_VIEWER_COLORBAR_HPP_
#define MSLAM_VIEWER_COLORBAR_HPP_

#include <QWidget>
#include <qwidget.h>

class ColorBar : public QWidget
{
  public:
    ColorBar(QWidget* parent = nullptr);

  protected:
    void paintEvent(QPaintEvent* event) override;
};

#endif // MSLAM_VIEWER_COLORBAR_HPP_
