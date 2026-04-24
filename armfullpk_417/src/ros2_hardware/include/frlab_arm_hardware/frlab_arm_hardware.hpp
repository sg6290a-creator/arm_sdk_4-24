#ifndef FRLAB_ARM_HARDWARE__FRLAB_ARM_HARDWARE_HPP_
#define FRLAB_ARM_HARDWARE__FRLAB_ARM_HARDWARE_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arm_sdk/frlab_arm.hpp"

namespace frlab_arm_hardware
{

class FrlabArmHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(FrlabArmHardware)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    static constexpr const char* LOGGER = "FrlabArmHardware";

    arm_sdk::FrlabArm arm_;
    std::string can_interface_;
    double default_velocity_rads_ = 0.5;
    std::array<double, arm_sdk::ARM_DOF> hw_positions_{};
    std::array<double, arm_sdk::ARM_DOF> hw_velocities_{};
    std::array<double, arm_sdk::ARM_DOF> hw_efforts_{};
    std::array<double, arm_sdk::ARM_DOF> hw_position_commands_{};
    rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};
    int read_deadline_ms_ = 3;
    int read_poll_timeout_ms_ = 1;
    std::size_t perf_log_every_n_cycles_ = 0;
    int consecutive_read_failures_ = 0;
    std::size_t perf_cycles_in_window_ = 0;
    std::size_t perf_read_failures_in_window_ = 0;
    double perf_read_ms_accum_ = 0.0;
    double perf_write_ms_accum_ = 0.0;
    double perf_cycle_ms_accum_ = 0.0;
    double last_read_ms_ = 0.0;
    static constexpr int kMaxReadFailures = 3;
};

}  // namespace frlab_arm_hardware

#endif  // FRLAB_ARM_HARDWARE__FRLAB_ARM_HARDWARE_HPP_
