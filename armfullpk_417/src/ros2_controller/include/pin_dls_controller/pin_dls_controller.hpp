#ifndef PIN_DLS_CONTROLLERS__JOINT_GROUP_DLS_CONTROLLER_HPP_
#define PIN_DLS_CONTROLLERS__JOINT_GROUP_DLS_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "forward_command_controller/forward_command_controller.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pin_dls_controller/pin_dls_solver.hpp"

namespace pin_dls_controllers
{

/**
 * \brief DLS inverse kinematics controller driven by a Cartesian pose goal.
 *
 * Subscribes to ~/target_pose (geometry_msgs/PoseStamped).
 * Each update step: FK → pose error → twist → DLS → joint position commands.
 *
 * \param joints             List of joint names to control.
 * \param end_effector_name  EE frame name in the URDF (e.g. "gripper_palm").
 * \param kp_pos             Position error gain  (default 1.0).
 * \param kp_rot             Rotation error gain  (default 1.0).
 * \param goal_tolerance     Stop threshold [m]   (default 0.005).
 */
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
    DLSSolver dls_solver_;
    Eigen::VectorXd q_current_;

    // Target pose (set from subscription)
    bool has_target_ = false;
    Eigen::Vector3d    target_pos_;
    Eigen::Quaterniond target_rot_;

    // Gains
    double kp_pos_ = 1.0;
    double kp_rot_ = 1.0;
    double goal_tolerance_ = 0.005;  // metres

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

}  // namespace pin_dls_controllers

#endif  // PIN_DLS_CONTROLLERS__JOINT_GROUP_DLS_CONTROLLER_HPP_
