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
    void scaleDrawnImage(int percent) { scaleDrawnImage(static_cast<float>(percent)); }
    void drawImage(const mslam::DepthFrame& newDepth);

  protected:
    void scaleDrawnImage(float percent);
    void initialize();

    float minDepth() { return static_cast<float>(ui.minSlider->value()) * 0.01f; }
    float maxDepth() { return static_cast<float>(ui.maxSlider->value()) * 0.01f; }

    Ui::DepthImageViewer ui;
    QImage image;
};

#endif // MSLAM_VIEWER_DEPTH_VIEWER_HPP_
