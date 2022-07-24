#ifndef MSLAM_VIEWER_DEPTH_VIEWER_HPP_
#define MSLAM_VIEWER_DEPTH_VIEWER_HPP_

#include "modular_slam/depth_frame.hpp"
#include "ui_depth_image_viewer.h"
#include <QWidget>

class DepthImageViewer : public QWidget
{
    Q_OBJECT
  public:
    DepthImageViewer(QWidget* parent = nullptr) : QWidget(parent) { initialize(); }

  public slots:
    void scaleDrawnImage(int percent);
    void drawImage(const mslam::DepthFrame& newDepth);

  protected:
    void paintEvent(QPaintEvent* event) override;
    void initialize();

    int minDepth() { return ui.minSlider->value() * 10; }
    int maxDepth() { return ui.maxSlider->value() * 10; }

    Ui::DepthImageViewer ui;
    QImage image;
};

#endif // MSLAM_VIEWER_DEPTH_VIEWER_HPP_
