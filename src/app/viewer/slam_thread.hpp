#ifndef SLAM_THREAD_H_
#define SLAM_THREAD_H_

#include "modular_slam/depth_frame.hpp"
#include "modular_slam/realsense_camera.hpp"
#include "modular_slam/rgbd_frame.hpp"
#include "modular_slam/slam.hpp"
#include "modular_slam/slam3d_types.hpp"
#include "pointcloud_viewer.hpp"
#include "slam_statistics.hpp"
#include <QImage>
#include <QPixmap>
#include <QThread>
#include <QVector3D>
#include <QVector>
#include <glm/fwd.hpp>
#include <glm/vec3.hpp>

class SlamThread : public QThread
{
    Q_OBJECT

  public:
    explicit SlamThread(QObject* parent);
    void setSlam(std::unique_ptr<mslam::Slam<mslam::RgbdFrame, mslam::slam3d::SensorState, Eigen::Vector3f>>&& newSlam)
    {
        slam = std::move(newSlam);
    }

  public slots:
    void stop() { isRunning = false; }
    void pause() { isPaused = true; }
    void resume() { isPaused = false; }

  private:
    void run() override;

  signals:
    void newRgbImageAvailable(const QImage& pixmap);
    void newDepthImageAvailable(const mslam::DepthFrame& depth);
    void newPointsAvailable(const std::vector<glm::vec3>& points);
    void newSlamStatisticsAvailable(const SlamStatistics& stats);
    void keyframeAdded(const KeyframeViewData& keyframe);
    void currentFrameChanged(const KeyframeViewData& keyframe);

  private:
    std::atomic<bool> isRunning;
    std::atomic<bool> isPaused;
    std::unique_ptr<mslam::Slam<mslam::RgbdFrame, mslam::slam3d::SensorState, Eigen::Vector3f>> slam;
    std::shared_ptr<mslam::RgbdFrame> frame;
};

#endif // SLAM_THREAD_H_
