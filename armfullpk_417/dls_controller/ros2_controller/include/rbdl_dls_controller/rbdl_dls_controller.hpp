#ifndef RBDL_DLS_CONTROLLERS__JOINT_GROUP_DLS_CONTROLLER_HPP_
#define RBDL_DLS_CONTROLLERS__JOINT_GROUP_DLS_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "forward_command_controller/forward_command_controller.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rbdl_dls_controller/rbdl_dls_solver.hpp"

namespace rbdl_dls_controllers
{

class JointGroupDLSController : public forward_command_controller::ForwardCommandController
{
public:
    JointGroupDLSController();

    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
    rbdl_dls_solver::DLSSolver dls_solver_;
    Eigen::VectorXd q_current_;

    bool has_target_ = false;
    Eigen::Vector3d    target_pos_;
    Eigen::Quaterniond target_rot_;

    double kp_pos_ = 1.0;
    double kp_rot_ = 1.0;
    double goal_tolerance_ = 0.005;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

}  // namespace rbdl_dls_controllers

#endif  // RBDL_DLS_CONTROLLERS__JOINT_GROUP_DLS_CONTROLLER_HPP_
