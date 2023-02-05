#include "slam_thread.hpp"
#include "modular_slam/basic_types.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/rgbd_frame.hpp"
#include "modular_slam/rgbd_slam_types.hpp"
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
#include <qmatrix4x4.h>

QMatrix4x4 poseToQMatrix(const mslam::slam3d::SensorState& state)
{
    QMatrix4x4 pose;

    Eigen::Matrix3f R = state.orientation.toRotationMatrix();
    Eigen::Vector3f T = state.position;
    Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
    transformMatrix.block<3, 3>(0, 0) = R;
    transformMatrix.block<3, 1>(0, 3) = T;

    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            pose(i, j) = transformMatrix(i, j);

    // pose.rotate(180, 1, 0, 0);
    return pose;
}

glm::mat4 poseToGlmMat4(const mslam::slam3d::SensorState& state)
{
    auto pose = glm::mat4(1);

    const Eigen::Matrix3f R = state.orientation.toRotationMatrix();
    const Eigen::Vector3f T = state.position;
    Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
    transformMatrix.block<3, 3>(0, 0) = R;
    transformMatrix.block<3, 1>(0, 3) = T;

    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            pose[j][i] = transformMatrix(i, j);

    return pose;
}

class FrontendVisitor : public mslam::KeyframeVisitor<mslam::slam3d::SensorState>,
                        public mslam::LandmarkVisitor<mslam::Vector3>

{
  public:
    virtual void visit(std::shared_ptr<mslam::Landmark<mslam::Vector3>>&) override;
    virtual void visit(std::shared_ptr<mslam::Keyframe<mslam::slam3d::SensorState>>&) override;

    void clearFlags() { keyframeAdded = nullptr; }
    std::shared_ptr<mslam::Keyframe<mslam::slam3d::SensorState>> recentlyKeyframeAdded() const;

  private:
    std::set<std::shared_ptr<mslam::Landmark<mslam::Vector3>>> landmarks;
    std::set<std::shared_ptr<mslam::Keyframe<mslam::slam3d::SensorState>>> keyframes;
    std::shared_ptr<mslam::Keyframe<mslam::slam3d::SensorState>> keyframeAdded;
};

void FrontendVisitor::visit(std::shared_ptr<mslam::Landmark<mslam::Vector3>>& landmark)
{
    landmarks.insert(landmark);
}

void FrontendVisitor::visit(std::shared_ptr<mslam::Keyframe<mslam::slam3d::SensorState>>& keyframe)
{
    if(keyframes.find(keyframe) == std::end(keyframes))
    {
        keyframeAdded = keyframe;
        keyframes.insert(keyframe);
    }
}

SlamThread::SlamThread(QObject* parent) : QThread(parent), isRunning(false), isPaused(false) {}

std::shared_ptr<mslam::Keyframe<mslam::slam3d::SensorState>> FrontendVisitor::recentlyKeyframeAdded() const
{
    return keyframeAdded;
}

void pointCloudFromRgbd(const mslam::RgbdFrame& rgbd, const mslam::rgbd::SensorState& currentPose,
                        std::vector<glm::vec3>& outPoints)
{
    const auto& depthFrame = rgbd.depth;
    const auto xScale = 1 / depthFrame.cameraParameters.focal.x();
    const auto yScale = 1 / depthFrame.cameraParameters.focal.y();
    const auto pose = poseToGlmMat4(currentPose);

    constexpr float invMultiplier = 1 / 255.f;
    const auto toGl =
        // glm::rotate(glm::identity<glm::mat4>(), glm::pi<float>(), glm::vec3(1.f, 0.f, 0.f));
        glm::eulerAngleZY(glm::radians(180.f), glm::radians(180.f));
    // glm::scale(glm::vec3(1.f, -1.f, 1.f));

    std::size_t outIndex = 0;
    for(auto u = 0; u < depthFrame.size.width; ++u)
    {
        for(auto v = 0; v < depthFrame.size.height; ++v)
        {
            const auto z = mslam::getDepth(depthFrame, {u, v});
            const auto isValid = mslam::isDepthValid(z);
            const auto x = (static_cast<float>(u) - depthFrame.cameraParameters.principalPoint.x()) * z * xScale;
            const auto y = (static_cast<float>(v) - depthFrame.cameraParameters.principalPoint.y()) * z * yScale;

            const std::size_t index = static_cast<std::size_t>(3 * (v * depthFrame.size.width + u));
            const auto b = rgbd.rgb.data[index] * invMultiplier;
            const auto g = rgbd.rgb.data[index + 1] * invMultiplier;
            const auto r = rgbd.rgb.data[index + 2] * invMultiplier;

            const glm::vec4 point = toGl * glm::vec4{x, y, z, 1.0f};
            const glm::vec3 rgb{r, g, b};

            outPoints[outIndex] = pose * point;
            outIndex += isValid;
            outPoints[outIndex] = rgb;
            outIndex += isValid;
        }
    }
}

void SlamThread::run()
{
    isRunning = true;
    const auto dataProvider = m_slam->dataProvider;
    auto frontend = m_slam->frontend;

    m_slam->init();

    std::vector<glm::vec3> points;
    SlamStatistics stats;
    FrontendVisitor visitor;

    while(isRunning)
    {
        while(isPaused)
        {
            msleep(5);
        }

        auto start = std::chrono::high_resolution_clock::now();
        m_slam->process();
        auto stop = std::chrono::high_resolution_clock::now();

        auto rgbd = dataProvider->recentData();
        if(points.size() < 2 * rgbd->depth.data.size())
            points.resize(2 * rgbd->depth.data.size());

        QImage originalImage{rgbd->rgb.data.data(), rgbd->rgb.size.width, rgbd->rgb.size.height,
                             3 * rgbd->rgb.size.width, QImage::Format_BGR888};
        const QImage image = originalImage.copy();

        emit newRgbImageAvailable(image);
        emit newDepthImageAvailable(rgbd->depth);

        pointCloudFromRgbd(*rgbd, m_slam->currentState(), points);

        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();
        stats.msPerFrame = static_cast<float>(microseconds) / 1000.0f;
        emit newSlamStatisticsAvailable(stats);
        emit newPointsAvailable(points);

        visitor.clearFlags();
        frontend->visitKeyframes(visitor);

        if(auto keyframe = visitor.recentlyKeyframeAdded(); keyframe != nullptr)
        {
            auto keyframePose = poseToQMatrix(keyframe->state);

            KeyframeViewData keyframeView = {image, keyframePose};
            emit keyframeAdded(keyframeView);
        }
    }
}
