#include <string>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pin_dls_controller/pin_dls_controller.hpp"
#include "rclcpp/parameter.hpp"


namespace pin_dls_controllers
{

JointGroupDLSController::JointGroupDLSController()
: forward_command_controller::ForwardCommandController()
{
    interface_name_ = hardware_interface::HW_IF_POSITION;
}


controller_interface::CallbackReturn JointGroupDLSController::on_init()
{
    auto ret = forward_command_controller::ForwardCommandController::on_init();
    if (ret != CallbackReturn::SUCCESS) {
        return ret;
    }

    try {
        auto_declare<std::string>("robot_description",    "");
        auto_declare<std::string>("kinematics_description", "");
        auto_declare<std::string>("end_effector_name", "gripper_palm");
        auto_declare<double>("kp_pos",          1.0);
        auto_declare<double>("kp_rot",          1.0);
        auto_declare<double>("goal_tolerance",  0.005);
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn JointGroupDLSController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    auto ret = forward_command_controller::ForwardCommandController::on_configure(previous_state);
    if (ret != CallbackReturn::SUCCESS) {
        return ret;
    }

    std::string urdf_string, ee_name;
    try {
        urdf_string = get_node()->get_parameter("kinematics_description").as_string();
        if (urdf_string.empty())
            urdf_string = get_node()->get_parameter("robot_description").as_string();
        ee_name = get_node()->get_parameter("end_effector_name").as_string();
        kp_pos_        = get_node()->get_parameter("kp_pos").as_double();
        kp_rot_        = get_node()->get_parameter("kp_rot").as_double();
        goal_tolerance_= get_node()->get_parameter("goal_tolerance").as_double();
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to get parameters: %s", e.what());
        return CallbackReturn::ERROR;
    }

    if (urdf_string.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "robot_description is empty");
        return CallbackReturn::ERROR;
    }

    if (!dls_solver_.init(urdf_string, ee_name)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize DLS solver");
        return CallbackReturn::ERROR;
    }

    q_current_.resize(static_cast<Eigen::Index>(params_.joints.size()));
    q_current_.setZero();

    pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/target_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            target_pos_ = Eigen::Vector3d(
                msg->pose.position.x,
                msg->pose.position.y,
                msg->pose.position.z);
            target_rot_ = Eigen::Quaterniond(
                msg->pose.orientation.w,
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z).normalized();
            has_target_ = true;
            RCLCPP_INFO(get_node()->get_logger(),
                "New target: [%.3f, %.3f, %.3f]",
                target_pos_.x(), target_pos_.y(), target_pos_.z());
        });

    RCLCPP_INFO(get_node()->get_logger(),
        "DLS Controller configured — EE: %s, kp_pos: %.2f, kp_rot: %.2f",
        ee_name.c_str(), kp_pos_, kp_rot_);
    return CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration
JointGroupDLSController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : params_.joints) {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
}


controller_interface::return_type JointGroupDLSController::update(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & period)
{
    if (!has_target_) {
        return controller_interface::return_type::OK;
    }

    // 1. 현재 관절 위치 읽기
    for (size_t i = 0; i < state_interfaces_.size(); ++i) {
        q_current_(static_cast<Eigen::Index>(i)) = state_interfaces_[i].get_value();
    }

    // 2. FK로 현재 EE 포즈 계산
    Eigen::Isometry3d current_pose = dls_solver_.eePose(q_current_);

    // 3. 포즈 에러 계산 (world frame)
    Eigen::Vector3d pos_err = target_pos_ - current_pose.translation();

    // 목표 도달 확인
    if (pos_err.norm() < goal_tolerance_) {
        return controller_interface::return_type::OK;
    }

    // 회전 에러: R_err = R_target * R_current^T → axis-angle
    Eigen::Matrix3d R_err = target_rot_.toRotationMatrix() * current_pose.linear().transpose();
    Eigen::AngleAxisd aa(R_err);
    Eigen::Vector3d rot_err = aa.angle() * aa.axis();

    // 4. 에러 → twist (비례 제어)
    Eigen::VectorXd v_t(6);
    v_t << kp_pos_ * pos_err, kp_rot_ * rot_err;

    // 5. DLS IK → 새 관절 위치
    Eigen::VectorXd q_new = dls_solver_.calculate(q_current_, v_t, period.seconds());

    // 6. 커맨드 출력
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
        command_interfaces_[i].set_value(q_new(static_cast<Eigen::Index>(i)));
    }

    return controller_interface::return_type::OK;
}

}  // namespace pin_dls_controllers


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    pin_dls_controllers::JointGroupDLSController,
    controller_interface::ControllerInterface)
