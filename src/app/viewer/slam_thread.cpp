#include "slam_thread.hpp"
#include "modular_slam/basic_types.hpp"
#include "modular_slam/frontend_interface.hpp"
#include "modular_slam/slam3d_types.hpp"
#include "pointcloud_viewer.hpp"
#include "slam_statistics.hpp"
#include <bits/chrono.h>
#include <chrono>
#include <qmatrix4x4.h>

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

std::shared_ptr<mslam::Keyframe<mslam::slam3d::SensorState>> FrontendVisitor::recentlyKeyframeAdded() const
{
    return keyframeAdded;
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
        auto start = std::chrono::high_resolution_clock::now();
        m_slam->process();
        auto stop = std::chrono::high_resolution_clock::now();

        auto rgbd = dataProvider->recentData();
        if(points.size() < 2 * rgbd->depth.data.size())
            points.resize(2 * rgbd->depth.data.size());

        QImage image{rgbd->rgb.data.data(), rgbd->rgb.size.width, rgbd->rgb.size.height, 3 * rgbd->rgb.size.width,
                     QImage::Format_BGR888};

        emit newRgbImageAvailable(image);
        emit newDepthImageAvailable(rgbd->depth);

        m_frame = rgbd;

        const auto& depthFrame = rgbd->depth;
        const auto xScale = 1 / depthFrame.cameraParameters.focal.x();
        const auto yScale = 1 / depthFrame.cameraParameters.focal.y();

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
                const auto b = rgbd->rgb.data[index] / 255.f;
                const auto g = rgbd->rgb.data[index + 1] / 255.f;
                const auto r = rgbd->rgb.data[index + 2] / 255.f;

                const glm::vec3 point{x, y, z};
                const glm::vec3 rgb{r, g, b};

                points[outIndex] = point;
                outIndex += isValid;
                points[outIndex] = rgb;
                outIndex += isValid;
            }
        }

        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();
        stats.msPerFrame = static_cast<float>(microseconds) / 1000.0f;
        emit newSlamStatisticsAvailable(stats);
        emit newPointsAvailable(points);

        visitor.clearFlags();
        frontend->visitKeyframes(visitor);

        if(auto keyframe = visitor.recentlyKeyframeAdded(); keyframe != nullptr)
        {
            QMatrix4x4 pose;
            pose.setToIdentity();

            Eigen::Matrix3f R = keyframe->state.orientation.toRotationMatrix();
            Eigen::Vector3f T = keyframe->state.position;
            Eigen::Matrix4f Trans; // Your Transformation Matrix
            Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
            Trans.block<3, 3>(0, 0) = R;
            Trans.block<3, 1>(0, 3) = T;

            for(int i = 0; i < 4; i++)
                for(int j = 0; j < 4; j++)
                    pose(i, j) = Trans(i, j);

            qDebug() << pose;

            KeyframeViewData keyframeView = {image, pose};

            emit keyframeAdded(keyframeView);
        }
    }
}
