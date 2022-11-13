#ifndef VIEWER_MAIN_WINDOW_HPP_
#define VIEWER_MAIN_WINDOW_HPP_

#include "modular_slam/depth_frame.hpp"
#include "pointcloud_viewer.hpp"
#include "slam_statistics.hpp"
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
    void setImage(const QImage& newImage) { ui->imageViewer->drawImage(newImage); }
    void setDepthImage(const mslam::DepthFrame& newImage) { ui->depthImageViewer->drawImage(newImage); }
    void setPoints(const std::vector<glm::vec3>& newPoints) { ui->pointcloudViewer->setPoints(newPoints); }
    void setSlamStatistics(const SlamStatistics& newSlamStats) { ui->slamStatsViewer->setSlamStatistics(newSlamStats); }
    void addKeyframe(const KeyframeViewData& keyframe) { ui->pointcloudViewer->addKeyframe(keyframe); }

  private:
    std::unique_ptr<Ui::ViewerMainWindow> ui;
};
} // namespace mslam

#endif // VIEWER_MAIN_WINDOW_HPP_
