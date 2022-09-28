#include "modular_slam/ceres_backend.hpp"
#include "modular_slam/constraints_interface.hpp"

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>
#include <ceres/types.h>

namespace mslam
{

struct SnavelyReprojectionError
{
    SnavelyReprojectionError(double observed_x, double observed_y) : observed_x(observed_x), observed_y(observed_y) {}

    template <typename T>
    bool operator()(const T* const camera, const T* const point, T* residuals) const
    {
        // camera[0,1,2] are the angle-axis rotation.
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point, p);
        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = -p[0] / p[2];
        T yp = -p[1] / p[2];

        // Apply second and fourth order radial distortion.
        const T& l1 = camera[7];
        const T& l2 = camera[8];
        T r2 = xp * xp + yp * yp;
        T distortion = 1.0 + r2 * (l1 + l2 * r2);

        // Compute final projected point position.
        const T& focal = camera[6];
        T predicted_x = focal * distortion * xp;
        T predicted_y = focal * distortion * yp;

        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        return true;
    }

    static ceres::CostFunction* Create(const double observed_x, const double observed_y)
    {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
            new SnavelyReprojectionError(observed_x, observed_y)));
    }

    double observed_x;
    double observed_y;
};

class CeresVisitor : public LandmarkConstraintVisitor<slam3d::SensorState, Vector3>,
                     public KeyframeConstraintVisitor<slam3d::SensorState, Vector3>
{
  public:
    CeresVisitor();
    void visit(const LandmarkConstraint<slam3d::SensorState, Vector3>&) override;
    void visit(const KeyframeConstraint<slam3d::SensorState, Vector3>&) override;

    void reset() { problem.reset(new ceres::Problem()); }

    std::shared_ptr<ceres::Problem> getProblem() { return problem; }

  private:
    std::shared_ptr<ceres::Problem> problem;
};

CeresVisitor::CeresVisitor() : problem(new ceres::Problem()) {}

void CeresVisitor::visit(const LandmarkConstraint<slam3d::SensorState, Vector3>& /*constraint*/)
{
    // ceres::CostFunction* costFunction = SnavelyReprojectionError::Create(0, 0);
    // problem.AddResidualBlock(cost_function, nullptr, &x);
}

void CeresVisitor::visit(const KeyframeConstraint<slam3d::SensorState, Vector3>& /*constraint*/)
{
    // ceres::CostFunction* cost_function =
    //     new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 1, 1>(new SnavelyReprojectionError(0, 0));
}

void CeresBackend::optimize(ConstraintsInterface<mslam::slam3d::SensorState, mslam::Vector3>& constraints)
{
    CeresVisitor visitor;

    constraints.visitKeyframeConstraints(visitor);
    constraints.visitLandmarkConstraints(visitor);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.logging_type = ceres::LoggingType::SILENT;
    ceres::Solver::Summary summary;

    auto problem = visitor.getProblem();
    ceres::Solve(options, problem.get(), &summary);
}

} // namespace mslam
