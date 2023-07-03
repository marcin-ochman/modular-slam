#include "slam_thread.hpp"
#include "image_viewer.hpp"
#include "modular_slam/basic_types.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/map_interface.hpp"
#include "modular_slam/projection.hpp"
#include "modular_slam/rgbd_frame.hpp"
#include "modular_slam/rgbd_slam_types.hpp"
#include "modular_slam/slam.hpp"
#include "modular_slam/slam3d_types.hpp"
#include "pointcloud_viewer.hpp"
#include "slam_statistics.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <bits/chrono.h>
#include <chrono>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/scalar_constants.hpp>
#include <glm/fwd.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/matrix.hpp>
#include <iterator>
#include <qpoint.h>
#include <unordered_set>

QMatrix4x4 poseToQMatrix(const mslam::slam3d::SensorState& state)
{
    QMatrix4x4 pose;

    const auto R = state.orientation.toRotationMatrix();
    const auto& T = state.position;
    mslam::Matrix4 transformMatrix = mslam::Matrix4::Identity();
    transformMatrix.block<3, 3>(0, 0) = R;
    transformMatrix.block<3, 1>(0, 3) = T;

    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            pose(i, j) = static_cast<float>(transformMatrix(i, j));

    return pose;
}

glm::mat4 poseToGlmMat4(const mslam::slam3d::SensorState& state)
{
    auto pose = glm::mat4(1);

    const mslam::Matrix3 R = state.orientation.toRotationMatrix();
    const mslam::Vector3& T = state.position;

    mslam::Matrix4 transformMatrix = mslam::Matrix4::Identity();
    transformMatrix.block<3, 3>(0, 0) = R;
    transformMatrix.block<3, 1>(0, 3) = T;

    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            pose[j][i] = static_cast<float>(transformMatrix(i, j));

    return pose;
}

class MapVisitor
    : public mslam::IMapVisitor<mslam::slam3d::SensorState, mslam::slam3d::LandmarkState, mslam::rgbd::RgbdKeypoint>
{
  public:
    void visit(std::shared_ptr<mslam::slam3d::Landmark> landmark) override;
    void visit(std::shared_ptr<mslam::slam3d::Keyframe> keyframe) override;

    void clearFlags() { keyframeAdded = nullptr; }
    std::shared_ptr<mslam::Keyframe<mslam::slam3d::SensorState>> recentlyKeyframeAdded() const;
    std::size_t keyframesCount() const { return keyframes.size(); }
    std::size_t landmarksCount() const { return landmarks.size(); }

    const std::unordered_set<std::shared_ptr<mslam::Landmark<mslam::Vector3>>>& maplandmarks() const
    {
        return landmarks;
    }

    const std::unordered_set<std::shared_ptr<mslam::Keyframe<mslam::slam3d::SensorState>>>& mapKeyframes() const
    {
        return keyframes;
    }

  private:
    std::unordered_set<std::shared_ptr<mslam::Landmark<mslam::Vector3>>> landmarks;
    std::unordered_set<std::shared_ptr<mslam::Keyframe<mslam::slam3d::SensorState>>> keyframes;
    std::shared_ptr<mslam::Keyframe<mslam::slam3d::SensorState>> keyframeAdded;
};

void MapVisitor::visit(std::shared_ptr<mslam::slam3d::Landmark> landmark)
{
    landmarks.insert(landmark);
}

void MapVisitor::visit(std::shared_ptr<mslam::slam3d::Keyframe> keyframe)
{
    if(keyframes.find(keyframe) == std::end(keyframes))
    {
        keyframeAdded = keyframe;
    }

    keyframes.insert(keyframe);
}

SlamThread::SlamThread(QObject* parent) : QThread(parent), isRunning(false), isPaused(false) {}

std::shared_ptr<mslam::Keyframe<mslam::slam3d::SensorState>> MapVisitor::recentlyKeyframeAdded() const
{
    return keyframeAdded;
}

void pointCloudFromMapLandmarks(const std::unordered_set<std::shared_ptr<mslam::Landmark<mslam::Vector3>>>& landmarks,
                                std::vector<glm::vec3>& outPoints)
{

    const glm::vec3 rgb{255, 255, 255};

    outPoints.clear();
    for(const auto& landmark : landmarks)
    {
        const auto point = glm::vec4{landmark->state.x(), landmark->state.y(), landmark->state.z(), 1.0f};

        outPoints.emplace_back(point);
        outPoints.push_back(rgb);
    }
}

void pointCloudFromRgbd(const mslam::RgbdFrame& rgbd, const mslam::rgbd::SensorState& currentPose,
                        std::vector<glm::vec3>& outPoints)
{
    const auto& depthFrame = rgbd.depth;
    const auto xScale = 1 / depthFrame.cameraParameters.focal.x();
    const auto yScale = 1 / depthFrame.cameraParameters.focal.y();
    const auto pose = poseToGlmMat4(currentPose);
    constexpr float invMultiplier = 1 / 255.f;
    const auto toGlPose = pose;

    std::size_t outIndex = 0;
    for(auto v = 0; v < depthFrame.size.height; ++v)
    {
        for(auto u = 0; u < depthFrame.size.width; ++u)
        {
            const auto z = mslam::getDepth(depthFrame, {u, v});
            const auto isValid = mslam::isDepthValid(z);
            const auto x = (static_cast<float>(u) - depthFrame.cameraParameters.principalPoint.x()) * z * xScale;
            const auto y = (static_cast<float>(v) - depthFrame.cameraParameters.principalPoint.y()) * z * yScale;

            const auto index = static_cast<std::size_t>(3 * (v * depthFrame.size.width + u));
            const auto b = rgbd.rgb.data[index] * invMultiplier;
            const auto g = rgbd.rgb.data[index + 1] * invMultiplier;
            const auto r = rgbd.rgb.data[index + 2] * invMultiplier;

            const glm::vec4 point = glm::vec4{x, y, z, 1.0f};
            const glm::vec3 rgb{r, g, b};

            outPoints[outIndex] = toGlPose * point;
            outIndex += isValid;
            outPoints[outIndex] = rgb;
            outIndex += isValid;
        }
    }

    outPoints.resize(outIndex);
}

void SlamThread::run()
{
    isRunning = true;

    slam->init();

    std::vector<glm::vec3> points, landmarkPoints;
    SlamStatistics stats;
    MapVisitor visitor;

    while(isRunning)
    {
        while(isPaused && !isInterruptionRequested())
        {
            msleep(5);
        }

        auto start = std::chrono::high_resolution_clock::now();
        auto result = slam->process();
        auto stop = std::chrono::high_resolution_clock::now();

        if(result == mslam::SlamProcessResult::NoDataAvailable)
        {
            isRunning = false;
            continue;
        }

        if(points.capacity() < rgbd->depth.data.size())
            points.reserve(2 * rgbd->depth.data.size());

        QImage originalImage{rgbd->rgb.data.data(), rgbd->rgb.size.width, rgbd->rgb.size.height,
                             3 * rgbd->rgb.size.width, QImage::Format_BGR888};
        const QImage image = originalImage.copy();

        emit depthImageChanged(rgbd->depth);

        auto state = slam->sensorState();
        QVector<QObservation> observations;
        const auto rotation = state.orientation.inverse();
        const mslam::Vector3 T = rotation * state.position;

        const auto& pp = rgbd->depth.cameraParameters.principalPoint;
        const auto& f = rgbd->depth.cameraParameters.focal;

        for(const auto& observation : landmarkObservations)
        {
            QObservation guiObservation;
            guiObservation.keypoint = QPoint(observation.observation.keypoint.coordinates.x(),
                                             observation.observation.keypoint.coordinates.y());

            auto point = mslam::projectOnImage(observation.landmark->state, rgbd->depth.cameraParameters, state);
            guiObservation.projectedLandmark = QPoint(point.x(), point.y());

            observations.push_back(guiObservation);
        }

        emit rgbImageChanged(image, observations);

        pointCloudFromRgbd(*rgbd, slam->sensorState(), points);

        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();
        stats.msPerFrame = static_cast<float>(microseconds) / 1000.0f;
        stats.keyframesCount = visitor.keyframesCount();
        stats.landmarksCount = visitor.landmarksCount();

        emit slamStatisticsChanged(stats);
        emit cameraPointsChanged(points);

        visitor.clearFlags();
        slam->map->visit(visitor);
        pointCloudFromMapLandmarks(visitor.maplandmarks(), landmarkPoints);

        if(auto keyframe = visitor.recentlyKeyframeAdded(); keyframe != nullptr)
        {
            auto keyframePose = poseToQMatrix(keyframe->state);

            KeyframeViewData keyframeView = {keyframe->id, image, keyframePose};
            spdlog::trace("Keyframe added {}", keyframe->id);
            emit keyframeAdded(keyframeView);
        }

        const auto pose = poseToQMatrix(slam->sensorState());
        KeyframeViewData currentFrameView = {0, image, pose};
        emit currentFrameChanged(currentFrameView);
        emit landmarkPointsChanged(landmarkPoints);

        if(isInterruptionRequested())
            isRunning = false;
    }
}
