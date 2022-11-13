#ifndef SLAM_THREAD_H_
#define SLAM_THREAD_H_

#include <QImage>
#include <QPixmap>
#include <QThread>
#include <QVector3D>
#include <QVector>
#include <glm/fwd.hpp>
#include <glm/vec3.hpp>
#include <set>

#include "modular_slam/depth_frame.hpp"
#include "modular_slam/keyframe.hpp"
#include "modular_slam/realsense_camera.hpp"
#include "modular_slam/rgbd_frame.hpp"
#include "modular_slam/slam.hpp"
#include "modular_slam/slam3d_types.hpp"
#include "pointcloud_viewer.hpp"
#include "slam_statistics.hpp"

#include <QDebug>
#include <qobject.h>

class SlamThread : public QThread
{
    Q_OBJECT

  public:
    SlamThread(QObject* parent) : QThread(parent) {}
    void setSlam(std::unique_ptr<mslam::Slam<mslam::RgbdFrame, mslam::slam3d::SensorState, Eigen::Vector3f>>&& newSlam)
    {
        m_slam = std::move(newSlam);
    }

  public slots:
    void stop() { isRunning = false; }

  private:
    void run() override;

  signals:
    void newRgbImageAvailable(const QImage& pixmap);
    void newDepthImageAvailable(const mslam::DepthFrame& depth);
    void newPointsAvailable(const std::vector<glm::vec3>& points);
    void newSlamStatisticsAvailable(const SlamStatistics& stats);
    void keyframeAdded(const KeyframeViewData& keyframe);

  private:
    bool isRunning;
    std::unique_ptr<mslam::Slam<mslam::RgbdFrame, mslam::slam3d::SensorState, Eigen::Vector3f>> m_slam;
    std::shared_ptr<mslam::RgbdFrame> m_frame;
};

#endif // SLAM_THREAD_H_
