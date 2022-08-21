#include "modular_slam/frontend/rgbd_feature_frontend.hpp"
#include "modular_slam/basic_types.hpp"
#include "modular_slam/camera.hpp"
#include "modular_slam/camera_parameters.hpp"
#include "modular_slam/constraints_interface.hpp"
#include "modular_slam/depth_frame.hpp"
#include "modular_slam/keyframe.hpp"
#include "modular_slam/landmark.hpp"
#include "modular_slam/observation.hpp"
#include "modular_slam/rgbd_frame.hpp"
#include "modular_slam/slam3d_types.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <memory>
#include <opencv2/core/hal/interface.h>
#include <opencv2/highgui.hpp>
#include <unordered_map>

namespace mslam
{

class BasicConstraints : public ConstraintsInterface<slam3d::State, Vector3d>
{
  public:
    void addConstraint(const LandmarkConstraint<slam3d::State, Vector3d> constraint) override;
    void addConstraint(const KeyframeConstraint<slam3d::State, Vector3d> constraint) override;

    void visitLandmarkConstraints(LandmarkConstraintVisitor<slam3d::State, Vector3d>& visitor) override;
    void visitKeyframeConstraints(KeyframeConstraintVisitor<slam3d::State, Vector3d>& visitor) override;

  private:
    std::vector<LandmarkConstraint<slam3d::State, Vector3d>> landmarkConstraints;
    std::vector<KeyframeConstraint<slam3d::State, Vector3d>> keyframeConstraints;
};

void BasicConstraints::addConstraint(const LandmarkConstraint<slam3d::State, Vector3d> constraint)
{
    landmarkConstraints.push_back(constraint);
}

void BasicConstraints::addConstraint(const KeyframeConstraint<slam3d::State, Vector3d> constraint)
{
    keyframeConstraints.push_back(constraint);
}

void BasicConstraints::visitLandmarkConstraints(LandmarkConstraintVisitor<slam3d::State, Vector3d>& visitor)
{
    for(const auto& constraint : landmarkConstraints)
        visitor.visit(constraint);
}

void BasicConstraints::visitKeyframeConstraints(KeyframeConstraintVisitor<slam3d::State, Vector3d>& visitor)
{
    for(const auto& constraint : keyframeConstraints)
        visitor.visit(constraint);
}

/*!
 * \brief Calculates 3D points in camera coordinate system based on points in image coordinate system
 */
std::unordered_map<int, Vector3d> pointsFromRgbdKeypoints(const DepthFrame& depthFrame,
                                                          const std::vector<cv::KeyPoint>& keypoints)
{
    std::unordered_map<int, Vector3d> points;

    const auto xScale = 1 / depthFrame.cameraParameters.focal.x();
    const auto yScale = 1 / depthFrame.cameraParameters.focal.y();

    int index = 0;
    for(const auto& keypoint : keypoints)
    {
        const int u = keypoint.pt.x, v = keypoint.pt.y;
        const auto depth = getDepth(depthFrame, u, v);

        if(isDepthValid(depth))
        {
            const float z = depth * 0.001f;
            const auto x = (u - depthFrame.cameraParameters.principalPoint.x()) * z * xScale;
            const auto y = (v - depthFrame.cameraParameters.principalPoint.y()) * z * yScale;

            Vector3d point{x, y, z};
            points.emplace(index, point);
        }

        index++;
    }

    return points;
}

RgbdFeatureFrontend::RgbdFeatureFrontend()
{
    orbDetector = cv::ORB::create();
    constraints = std::make_shared<BasicConstraints>();
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
}

bool RgbdFeatureFrontend::isNewKeyframeRequired() const
{
    const auto minMatchedLandmarks = 30;

    int matchedLandmarks = 40;
    return matchedLandmarks < minMatchedLandmarks;
}

cv::Mat toCameraMatrix(const CameraParameters& cameraParameters)
{
    cv::Mat cameraMatrix =
        (cv::Mat_<double>(3, 3) << cameraParameters.focal.x(), 0, cameraParameters.principalPoint.x(), 0,
         cameraParameters.focal.y(), cameraParameters.principalPoint.y(), 0, 0, 1);

    return cameraMatrix;
}

std::shared_ptr<RgbdFeatureFrontend::Constraints> RgbdFeatureFrontend::prepareConstraints(const RgbdFrame& sensorData)
{
    cv::Mat bgr{sensorData.rgb.size.height, sensorData.rgb.size.width, CV_8UC3,
                const_cast<std::uint8_t*>(sensorData.rgb.data.data())};

    cv::Mat descriptors, bgrWithKeypoints;
    std::vector<cv::KeyPoint> keypoints;

    orbDetector->detectAndCompute(bgr, cv::Mat(), keypoints, descriptors);
    descriptors.convertTo(descriptors, CV_32F);

    std::vector<std::shared_ptr<Landmark<Vector3d>>> matchedLandmarks;

    if(referenceKeyframeData.keyframe != nullptr && descriptors.total() > 0 &&
       referenceKeyframeData.descriptors.total() > 0)
    {
        std::vector<std::vector<cv::DMatch>> matches;
        matcher->knnMatch(descriptors, referenceKeyframeData.descriptors, matches, 2);
        auto landmarksCoordinates = pointsFromRgbdKeypoints(sensorData.depth, keypoints);

        cv::Mat worldCoords(static_cast<int>(landmarksCoordinates.size()), 3, CV_32F),
            imagePoints(static_cast<int>(landmarksCoordinates.size()), 2, CV_32F);

        std::vector<cv::DMatch> goodMatches;

        for(size_t i = 0; i < matches.size(); i++)
        {
            if(matches[i][0].distance < 0.7 * matches[i][1].distance)
                goodMatches.push_back(matches[i][0]);
        }

        std::size_t pointsMatched = 0;
        for(std::size_t i = 0; i < goodMatches.size(); ++i)
        {
            const auto& match = goodMatches[i];
            if(landmarksCoordinates.count(match.queryIdx))
            {
                const auto& point = landmarksCoordinates[match.queryIdx];
                const auto& keypoint = keypoints[match.queryIdx];
                auto worldCoordsPtr = worldCoords.ptr<float>(pointsMatched);
                worldCoordsPtr[0] = point.x();
                worldCoordsPtr[1] = point.y();
                worldCoordsPtr[2] = point.z();

                auto imagePointsPtr = imagePoints.ptr<float>(pointsMatched++);
                imagePointsPtr[0] = keypoint.pt.x;
                imagePointsPtr[1] = keypoint.pt.y;

                auto landmark = std::make_shared<Landmark<Vector3d>>();
                landmark->state = point;

                matchedLandmarks.push_back(landmark);
            }
        }

        constexpr int MIN_POINTS_THRESHOLD = 10;
        if(pointsMatched > MIN_POINTS_THRESHOLD)
        {
            worldCoords = worldCoords(cv::Range(0, pointsMatched), cv::Range::all());
            imagePoints = imagePoints(cv::Range(0, pointsMatched), cv::Range::all());

            cv::Mat cameraMatrix = toCameraMatrix(sensorData.depth.cameraParameters);
            cv::Mat tVec, rot;

            cv::solvePnP(worldCoords, imagePoints, cameraMatrix, cv::Mat(), rot, tVec);
        }
    }

    if(isNewKeyframeRequired())
    {
        auto newKeyframe = std::make_shared<Keyframe<slam3d::State>>();

        for(const auto& landmark : matchedLandmarks)
        {
            LandmarkConstraint<slam3d::State, Vector3d> landmarkConstraint{newKeyframe, landmark, landmark->state};
            constraints->addConstraint(landmarkConstraint);
        }

        referenceKeyframeData.descriptors = descriptors;
        referenceKeyframeData.keypoints = std::move(keypoints);
        referenceKeyframeData.keyframe = std::move(newKeyframe);
    }

    return constraints;
}

} // namespace mslam
