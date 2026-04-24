#include "rbdl_dls_controller/rbdl_dls_solver.hpp"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

namespace rbdl_dls_solver
{

bool DLSSolver::init()
{
    auto I3 = [](double ixx, double iyy, double izz) {
        Matrix3d I = Matrix3d::Zero();
        I(0,0) = ixx; I(1,1) = iyy; I(2,2) = izz;
        return I;
    };

    Matrix3d R_base;
    R_base << -1, 0, 0,
               0,-1, 0,
               0, 0, 1;
    unsigned int base_id = model_.AddBody(0,
        SpatialTransform(R_base, Vector3d(0, 0, 0)),
        Joint(JointTypeFixed),
        Body(4.0, Vector3d::Zero(), I3(0.00443333156, 0.00443333156, 0.0072)),
        "base_link_inertia");

    unsigned int shoulder_id = model_.AddBody(base_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d(0, 0, 0.0596)),
        Joint(JointTypeRevolute, Vector3d::UnitZ()),
        Body(3.7, Vector3d::Zero(), I3(0.010267495893, 0.010267495893, 0.00666)),
        "shoulder_link");

    unsigned int upper_arm_id = model_.AddBody(shoulder_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d(0, -0.0606, 0.080)),
        Joint(JointTypeRevolute, Vector3d::UnitY()),
        Body(8.393, Vector3d::Zero(), I3(0.133885781862, 0.133885781862, 0.0151074)),
        "upper_arm_link");

    unsigned int forearm_id = model_.AddBody(upper_arm_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d(0, 0, 0.215)),
        Joint(JointTypeRevolute, Vector3d::UnitY()),
        Body(2.275, Vector3d::Zero(), I3(0.0312093550996, 0.0312093550996, 0.004095)),
        "forearm_link");

    unsigned int wrist1_id = model_.AddBody(forearm_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d(0, -0.001, 0.215)),
        Joint(JointTypeRevolute, Vector3d::UnitY()),
        Body(1.219, Vector3d::Zero(), I3(0.00255989897604, 0.00255989897604, 0.0021942)),
        "wrist_1_link");

    unsigned int wrist2_id = model_.AddBody(wrist1_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d(0, -0.0425, 0.0375)),
        Joint(JointTypeRevolute, Vector3d::UnitZ()),
        Body(1.219, Vector3d::Zero(), I3(0.00255989897604, 0.00255989897604, 0.0021942)),
        "wrist_2_link");

    unsigned int wrist3_id = model_.AddBody(wrist2_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d(0.0375, 0, 0.043)),
        Joint(JointTypeRevolute, Vector3d::UnitX()),
        Body(0.1, Vector3d::Zero(), I3(0.0001, 0.0001, 0.0001)),
        "wrist_3_link");

    ee_id_ = model_.AddBody(wrist3_id,
        SpatialTransform(Matrix3d::Identity(), Vector3d::Zero()),
        Joint(JointTypeFixed),
        Body(0.0, Vector3d(0,0,0), Vector3d(0,0,0)),
        "tool0");

    J_.resize(6, model_.dof_count);
    J_.setZero();
    return true;
}

Eigen::VectorXd DLSSolver::calculate(const Eigen::VectorXd& q, const Eigen::VectorXd& desired_v, double dt)
{
    J_.setZero();
    CalcPointJacobian6D(model_, q, ee_id_, Vector3d::Zero(), J_, true);

    Eigen::MatrixXd J_reordered(6, model_.dof_count);
    J_reordered << J_.bottomRows(3), J_.topRows(3);

    Eigen::MatrixXd J_T = J_reordered.transpose();
    Eigen::MatrixXd H = J_reordered * J_T + lambda_ * lambda_ * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd dq = J_T * H.inverse() * desired_v;

    return q + dq * dt;
}

Eigen::Isometry3d DLSSolver::eePose(const Eigen::VectorXd& q)
{
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = CalcBodyToBaseCoordinates(model_, q, ee_id_, Vector3d::Zero(), true);
    pose.linear() = CalcBodyWorldOrientation(model_, q, ee_id_, false).transpose();
    return pose;
}

}  // namespace rbdl_dls_solver
