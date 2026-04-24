#pragma once

#include <string>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <Eigen/Dense>

class DLSSolver {
public:
    bool init(const std::string& urdf_string, const std::string& ee_name);

    // DLS IK step: q_current + target velocity → q_next
    Eigen::VectorXd calculate(const Eigen::VectorXd& q, const Eigen::VectorXd& v_t, double dt);

    // Forward kinematics: returns EE pose (rotation | translation)
    Eigen::Isometry3d eePose(const Eigen::VectorXd& q);

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    Eigen::MatrixXd J_;
    pinocchio::FrameIndex ee_frame_id_;
    double lambda_ = 1e-3;
};
