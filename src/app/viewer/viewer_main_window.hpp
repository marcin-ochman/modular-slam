#ifndef VIEWER_MAIN_WINDOW_HPP_
#define VIEWER_MAIN_WINDOW_HPP_

#include "modular_slam/types/depth_frame.hpp"
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
    Q_OBJECT

  public:
    ViewerMainWindow(QWidget* parent = nullptr);

  public slots:
    void setImage(const QImage& newImage) { ui->imageViewer->drawImage(newImage); }
    void setImageWithObservations(const QImage& newImage, const QVector<QObservation>& observations)
    {
        ui->imageViewer->drawImageWithObservations(newImage, observations);
    }
    void setDepthImage(const mslam::DepthFrame& newImage) { ui->depthImageViewer->drawImage(newImage); }
    void setCurrentCameraPoints(const std::vector<glm::vec3>& newPoints)
    {
        ui->pointcloudViewer->setCurrentCameraPoints(newPoints);
    }

    void setLandmarkPoints(const std::vector<glm::vec3>& newPoints)
    {
        ui->pointcloudViewer->setLandmarkPoints(newPoints);
    }

    void setSlamStatistics(const SlamStatistics& newSlamStats) { ui->slamStatsViewer->setSlamStatistics(newSlamStats); }
    void addKeyframe(const KeyframeViewData& keyframe) { ui->pointcloudViewer->addKeyframe(keyframe); }
    void setCurrentFrame(const KeyframeViewData& keyframe) { ui->pointcloudViewer->setCurrentFrame(keyframe); }

  protected:
    void closeEvent(QCloseEvent* event) override;
    void save3dView();
    void saveRgbImage();

  signals:
    void paused();
    void resumed();
    void isClosing();

  private slots:
    void onPauseResume();

  private:
    void saveSettings();
    void loadSettings();
    bool isPaused = false;
    std::unique_ptr<Ui::ViewerMainWindow> ui;
};
} // namespace mslam

#endif // VIEWER_MAIN_WINDOW_HPP_
