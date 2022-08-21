#include "modular_slam/ceres_backend.hpp"
#include "modular_slam/constraints_interface.hpp"

#include <ceres/ceres.h>

namespace mslam
{

class CeresVisitor : public LandmarkConstraintVisitor<slam3d::State, Vector3d>,
                     public KeyframeConstraintVisitor<slam3d::State, Vector3d>
{
  public:
    void visit(const LandmarkConstraint<slam3d::State, Vector3d>&) override;
    void visit(const KeyframeConstraint<slam3d::State, Vector3d>&) override;

    std::unique_ptr<ceres::Problem> getProblem() { return nullptr; }
};

void CeresVisitor::visit(const LandmarkConstraint<slam3d::State, Vector3d>& constraint)
{
    // CostFunction* cost_function = new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    // problem.AddResidualBlock(cost_function, nullptr, &x);
}

void CeresVisitor::visit(const KeyframeConstraint<slam3d::State, Vector3d>& constraint)
{
    // CostFunction* cost_function = new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    // problem.AddResidualBlock(cost_function, nullptr, &x);
}

void CeresBackend::optimize(ConstraintsInterface<mslam::slam3d::State, mslam::Vector3d>& constraints)
{
    CeresVisitor visitor;

    constraints.visitKeyframeConstraints(visitor);
    constraints.visitLandmarkConstraints(visitor);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;

    auto problem = visitor.getProblem();

    ceres::Solve(options, problem.get(), &summary);
}

} // namespace mslam
