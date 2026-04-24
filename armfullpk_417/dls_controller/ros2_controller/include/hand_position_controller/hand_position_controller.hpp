#ifndef HAND_POSITION_CONTROLLER__HAND_POSITION_CONTROLLER_HPP_
#define HAND_POSITION_CONTROLLER__HAND_POSITION_CONTROLLER_HPP_

#include <array>
#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "core/gripper.hpp"
#include "core/motor_map.hpp"

namespace hand_position_controller
{

constexpr int HAND_DOF = 12;

class HandPositionController : public rclcpp::Node
{
public:
    explicit HandPositionController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~HandPositionController();

private:
    void onPreset(const std_msgs::msg::String::SharedPtr msg);
    bool loadPresets();
    void sendPosition(const std::array<int32_t, HAND_DOF> & positions);

    std::unique_ptr<gripper::Gripper> gripper_;
    std::map<std::string, std::array<int32_t, HAND_DOF>> presets_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr preset_sub_;
};

}  // namespace hand_position_controller

#endif  // HAND_POSITION_CONTROLLER__HAND_POSITION_CONTROLLER_HPP_
