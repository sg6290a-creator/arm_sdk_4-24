/**
 * ============================================================================
 * ArmDriver — N-Joint RMD Motor Driver (순수 C++)
 * ============================================================================
 *
 * USBCANDevice + RMDProtocol을 결합하여 N개 관절을 통합 관리.
 * ROS 의존성 없음. ros2_control SystemInterface에서 has-a로 사용.
 *
 * ============================================================================
 */

#ifndef ARM_SDK__ARM_DRIVER_HPP_
#define ARM_SDK__ARM_DRIVER_HPP_

#include "arm_sdk/usbcan_device.hpp"
#include "arm_sdk/rmd_protocol.hpp"

#include <cstdint>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>

namespace arm_sdk
{

/// Per-joint motor data
struct JointData {
    std::string joint_name;
    int         actuator_id      = 1;
    double      torque_constant  = 1.0;

    // State (read from motor)
    double position_state  = 0.0;
    double velocity_state  = 0.0;
    double effort_state    = 0.0;
    int8_t temperature     = 0;

    // Command (written to motor)
    double position_command  = 0.0;
    double velocity_command  = 0.0;
    double effort_command    = 0.0;
};

/// Driver configuration
struct ArmDriverConfig {
    // USBCAN settings
    uint32_t    device_type   = USBCAN_II;
    uint32_t    device_index  = 0;
    uint32_t    can_channel   = 0;
    int         baudrate      = 1000000;
    std::string lib_path      = "libusbcan.so";

    // Motor dynamics
    uint32_t motor_acceleration  = 500;    // dps²
    uint32_t motor_deceleration  = 500;    // dps²
    double   max_velocity_dps    = 360.0;  // deg/s
    double   default_velocity_dps = 50.0;  // deg/s

    // Joint definitions
    struct JointDef {
        std::string name;
        int  actuator_id     = 1;
        double torque_constant = 1.0;
    };
    std::vector<JointDef> joints;
};

/// Logging callback (optional, for forwarding to ROS logger etc.)
using LogCallback = std::function<void(int level, const std::string& msg)>;

#ifndef ARM_SDK_LOG_LEVELS_DEFINED
#define ARM_SDK_LOG_LEVELS_DEFINED
constexpr int LOG_INFO  = 0;
constexpr int LOG_WARN  = 1;
constexpr int LOG_ERROR = 2;
#endif

class ArmDriver
{
public:
    ArmDriver();
    ~ArmDriver();

    // Non-copyable
    ArmDriver(const ArmDriver&) = delete;
    ArmDriver& operator=(const ArmDriver&) = delete;

    /// Set logging callback (call before configure)
    void setLogCallback(LogCallback cb);

    // ── Lifecycle ──────────────────────────────────────────────

    /**
     * Configure: load library, open device, init CAN, test motors, set accel.
     * @return true on success
     */
    bool configure(const ArmDriverConfig& config);

    /**
     * Activate: read current positions, initialize commands.
     * @return true on success
     */
    bool activate();

    /**
     * Deactivate: stop motors (optional home).
     */
    void deactivate();

    /**
     * Cleanup: close device, unload library.
     */
    void cleanup();

    // ── Communication ──────────────────────────────────────────

    /**
     * Read all motors (batch: send all 0x9C, collect responses).
     * @return true if at least one motor responded
     */
    bool readAll();

    /**
     * Write all motor commands (batch: send all 0xA4).
     */
    void writeAll();

    /**
     * Read a single motor state.
     * @return true on success
     */
    bool readMotor(size_t index);

    /**
     * Write a single motor command.
     * @return true on success
     */
    bool writeMotor(size_t index);

    /**
     * Set acceleration for a motor.
     */
    bool setAcceleration(int actuator_id, uint32_t accel_dps2);

    /**
     * Clear CAN receive buffer.
     */
    void clearReceiveBuffer();

    // ── Joint Access ───────────────────────────────────────────

    size_t jointCount() const { return joints_.size(); }
    JointData& joint(size_t i) { return joints_[i]; }
    const JointData& joint(size_t i) const { return joints_[i]; }
    std::vector<JointData>& joints() { return joints_; }

    /// Thread-safe lock for state/command access
    std::mutex& mutex() { return mutex_; }

    // ── Holding Thread ─────────────────────────────────────────

    /**
     * Start a background thread that sends position-hold commands.
     * Used during deactivation to keep motors at home position.
     */
    void startHoldingThread();
    void stopHoldingThread();

private:
    void log(int level, const std::string& msg);

    USBCANDevice can_;
    ArmDriverConfig config_;
    std::vector<JointData> joints_;
    uint32_t can_channel_ = 0;

    std::mutex mutex_;
    LogCallback log_cb_;

    // Holding thread
    std::thread holding_thread_;
    std::atomic<bool> stop_holding_{true};
    void holdingThreadFunc();
};

}  // namespace arm_sdk

#endif  // ARM_SDK__ARM_DRIVER_HPP_
