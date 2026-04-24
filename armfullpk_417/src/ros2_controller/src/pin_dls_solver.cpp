#include "pin_dls_controller/pin_dls_solver.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>


bool DLSSolver::init(const std::string& urdf_string, const std::string& ee_name)
{
    try {
        pinocchio::urdf::buildModelFromXML(urdf_string, model_);
        data_ = pinocchio::Data(model_);
        ee_frame_id_ = model_.getFrameId(ee_name);
        if (ee_frame_id_ >= model_.frames.size()) {
            return false;
        }
        J_.resize(6, model_.nv);
        J_.setZero();
        return true;
    }
    catch (const std::exception& e) {
        return false;
    }
}


Eigen::VectorXd DLSSolver::calculate(const Eigen::VectorXd& q, const Eigen::VectorXd& v_t, double dt)
{
    pinocchio::computeFrameJacobian(
        model_, data_, q, ee_frame_id_,
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_);

    Eigen::MatrixXd J_T = J_.transpose();
    Eigen::MatrixXd H = J_ * J_T + lambda_ * lambda_ * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd dq = J_T * H.inverse() * v_t;

    return pinocchio::integrate(model_, q, dq * dt);
}


Eigen::Isometry3d DLSSolver::eePose(const Eigen::VectorXd& q)
{
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacement(model_, data_, ee_frame_id_);

    Eigen::Isometry3d pose;
    pose.linear()      = data_.oMf[ee_frame_id_].rotation();
    pose.translation() = data_.oMf[ee_frame_id_].translation();
    return pose;
}
