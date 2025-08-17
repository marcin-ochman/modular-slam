#ifndef SLAM_THREAD_H_
#define SLAM_THREAD_H_

#include "image_viewer.hpp"
#include "modular_slam/sensors/realsense_camera.hpp"
#include "modular_slam/slam.hpp"
#include "modular_slam/types/rgbd_frame.hpp"
#include "modular_slam/types/rgbd_slam_types.hpp"
#include "pointcloud_viewer.hpp"
#include "slam_statistics.hpp"
#include <QImage>
#include <QPixmap>
#include <QThread>
#include <QVector3D>
#include <QVector>
#include <glm/fwd.hpp>
#include <glm/vec3.hpp>

#include "range.hpp"

class SlamThread : public QThread
{
    Q_OBJECT

  public:
    explicit SlamThread(QObject* parent);
    void
    setSlam(std::unique_ptr<mslam::KeypointSlam<mslam::RgbdFrame, mslam::rgbd::SensorState, mslam::rgbd::LandmarkState,
                                                mslam::rgbd::RgbdOrbKeypointDescriptor>>&& newSlam)
    {
        slam = std::move(newSlam);
    }

    void setRecentFrame(std::shared_ptr<mslam::RgbdFrame> newFrame) { rgbd = std::move(newFrame); }
    void setRecentObservations(
        const std::vector<mslam::LandmarkObservation<mslam::slam3d::LandmarkState,
                                                     mslam::rgbd::RgbdOrbKeypointDescriptor>>& newObservations)
    {
        landmarkObservations = newObservations;
    }

  public slots:
    void stop() { isRunning = false; }
    void pause() { isPaused = true; }
    void resume() { isPaused = false; }

  private:
    void run() override;

  signals:
    void rgbImageChanged(const QImage& pixmap, const QVector<QObservation>& observations);
    void depthImageChanged(const mslam::DepthFrame& depth);
    void cameraPointsChanged(const std::vector<glm::vec3>& points);
    void landmarkPointsChanged(const std::vector<glm::vec3>& points);
    void slamStatisticsChanged(const SlamStatistics& stats);
    void keyframeAdded(const KeyframeViewData& keyframe);
    void currentFrameChanged(const KeyframeViewData& keyframe);

    void newRangeParameterRegistered(const QString& name, float value, const Range& range);
    void newChoiceParameterRegistered(const QString& name, const int, const QVector<int>& choices);

  private:
    std::atomic<bool> isRunning;
    std::atomic<bool> isPaused;
    std::unique_ptr<mslam::KeypointSlam<mslam::RgbdFrame, mslam::rgbd::SensorState, mslam::rgbd::LandmarkState,
                                        mslam::rgbd::RgbdOrbKeypointDescriptor>>
        slam;

    std::shared_ptr<mslam::RgbdFrame> rgbd;
    std::vector<mslam::LandmarkObservation<mslam::slam3d::LandmarkState, mslam::rgbd::RgbdOrbKeypointDescriptor>>
        landmarkObservations;
};

#endif // SLAM_THREAD_H_
