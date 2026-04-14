/**
 * ============================================================================
 * IntegratedDriver — RMD + Robstride 통합 모터 드라이버 (순수 C++)
 * ============================================================================
 *
 * 1개의 USBCAN 디바이스에서 RMD (Standard CAN) + Robstride (Extended CAN)
 * 모터를 혼합하여 순차 제어.
 *
 * 프레임 구분:
 *   - RMD:       extern_flag=0 (11-bit Standard CAN)
 *   - Robstride: extern_flag=1 (29-bit Extended CAN)
 *
 * 사용 예:
 *   IntegratedDriverConfig cfg;
 *   cfg.joints = {
 *       {"joint_1", MotorType::RMD,       1, 0.32, 0},
 *       {"joint_2", MotorType::RMD,       2, 0.32, 0},
 *       {"joint_3", MotorType::RMD,       3, 0.32, 0},
 *       {"joint_4", MotorType::RMD,       4, 0.32, 0},
 *       {"joint_5", MotorType::ROBSTRIDE, 1, 1.0,  5.0},
 *   };
 *   IntegratedDriver driver;
 *   driver.configure(cfg);
 *   driver.activate();
 *   driver.readAll();
 *   driver.writeAll();
 *
 * ============================================================================
 */

#ifndef ARM_SDK__INTEGRATED_DRIVER_HPP_
#define ARM_SDK__INTEGRATED_DRIVER_HPP_

#include "arm_sdk/usbcan_device.hpp"
#include "arm_sdk/rmd_protocol.hpp"
#include "arm_sdk/robstride_protocol.hpp"

#include <cstdint>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <functional>

namespace arm_sdk
{

// ════════════════════════════════════════════════════════════════
//  Motor Type
// ════════════════════════════════════════════════════════════════

enum class MotorType {
    RMD,        // MyActuator RMD — Standard CAN (11-bit)
    ROBSTRIDE   // Robstride RS01 — Extended CAN (29-bit)
};

inline const char* motorTypeString(MotorType t) {
    return (t == MotorType::RMD) ? "RMD" : "Robstride";
}

// ════════════════════════════════════════════════════════════════
//  Unified Joint
// ════════════════════════════════════════════════════════════════

/// Per-joint data for mixed motor types
struct UnifiedJoint {
    std::string name;
    MotorType   motor_type   = MotorType::RMD;
    int         motor_id     = 1;

    // Configuration
    double torque_constant     = 1.0;   // RMD: torque = current × constant
    float  robstride_max_speed = 5.0f;  // Robstride: position mode speed limit (rad/s)

    // State (읽기)
    double  position_rad   = 0.0;
    double  velocity_rads  = 0.0;
    double  effort_nm      = 0.0;
    double  temperature    = 0.0;
    uint8_t rs_mode        = 0;       // Robstride feedback mode
    uint8_t rs_error_bits  = 0;       // Robstride error bits
    bool    enabled        = false;

    // Command (쓰기)
    double position_command  = 0.0;
    double velocity_command  = 0.0;
};

// ════════════════════════════════════════════════════════════════
//  Configuration
// ════════════════════════════════════════════════════════════════

struct IntegratedDriverConfig {
    // USBCAN settings
    uint32_t    device_type   = USBCAN_II;
    uint32_t    device_index  = 0;
    uint32_t    can_channel   = 0;
    int         baudrate      = 1000000;
    std::string lib_path      = "libusbcan.so";

    // RMD motor dynamics
    uint32_t rmd_acceleration    = 500;     // dps²
    uint32_t rmd_deceleration    = 500;     // dps²
    double   rmd_max_vel_dps     = 360.0;   // deg/s
    double   rmd_default_vel_dps = 50.0;    // deg/s

    // Robstride defaults
    float    rs_limit_speed      = 5.0f;    // rad/s (position mode)
    float    rs_limit_torque     = 17.0f;   // Nm
    float    rs_limit_current    = 23.0f;   // A

    // Joint definitions
    struct JointDef {
        std::string name;
        MotorType   motor_type      = MotorType::RMD;
        int         motor_id        = 1;
        double      torque_constant = 1.0;   // RMD only
        float       max_speed_rads  = 5.0f;  // Robstride position mode
    };
    std::vector<JointDef> joints;
};

// ════════════════════════════════════════════════════════════════
//  Logging
// ════════════════════════════════════════════════════════════════

// Log levels (same as arm_driver.hpp)
#ifndef ARM_SDK_LOG_LEVELS_DEFINED
#define ARM_SDK_LOG_LEVELS_DEFINED
constexpr int LOG_INFO  = 0;
constexpr int LOG_WARN  = 1;
constexpr int LOG_ERROR = 2;
#endif

using IntegratedLogCallback = std::function<void(int level, const std::string& msg)>;

// ════════════════════════════════════════════════════════════════
//  IntegratedDriver
// ════════════════════════════════════════════════════════════════

class IntegratedDriver
{
public:
    IntegratedDriver();
    ~IntegratedDriver();

    // Non-copyable
    IntegratedDriver(const IntegratedDriver&) = delete;
    IntegratedDriver& operator=(const IntegratedDriver&) = delete;

    /// Set logging callback (call before configure)
    void setLogCallback(IntegratedLogCallback cb);

    // ── Lifecycle ──────────────────────────────────────────────

    /**
     * Configure: load USBCAN, open device, init CAN, test all motors.
     * RMD: send 0x9C read → check response
     * Robstride: send Enable → check feedback
     * @return true on success
     */
    bool configure(const IntegratedDriverConfig& config);

    /**
     * Activate: enable all motors, read initial positions.
     * RMD: set acceleration, read positions
     * Robstride: set position mode, enable, read mech_pos
     * @return true on success
     */
    bool activate();

    /**
     * Deactivate: disable all motors safely.
     */
    void deactivate();

    /**
     * Cleanup: close USBCAN, unload library.
     */
    void cleanup();

    // ── Communication ──────────────────────────────────────────

    /**
     * Read all motor states (순차적: RMD → Robstride).
     * @return true if at least one motor responded
     */
    bool readAll();

    /**
     * Write all motor commands (순차적).
     * RMD: position control (0xA4)
     * Robstride: write loc_ref parameter
     */
    void writeAll();

    /**
     * Read single motor state.
     * @return true on success
     */
    bool readMotor(size_t index);

    /**
     * Write single motor command.
     * @return true on success
     */
    bool writeMotor(size_t index);

    // ── Robstride specific ─────────────────────────────────────

    /**
     * Enable a Robstride motor.
     * @return true if feedback received
     */
    bool enableRobstride(size_t index);

    /**
     * Disable a Robstride motor.
     */
    bool disableRobstride(size_t index);

    /**
     * Set Robstride run mode (requires disable → set → enable).
     */
    bool setRobstrideMode(size_t index, uint8_t mode);

    /**
     * Read a Robstride parameter.
     */
    bool readRobstrideParam(size_t index, uint16_t param_id, float& value);

    /**
     * Write a Robstride parameter.
     */
    bool writeRobstrideParam(size_t index, uint16_t param_id, float value);

    // ── RMD specific ───────────────────────────────────────────

    /**
     * Set RMD acceleration.
     */
    bool setRmdAcceleration(size_t index, uint32_t accel_dps2);

    // ── Joint Access ───────────────────────────────────────────

    size_t jointCount() const { return joints_.size(); }
    UnifiedJoint& joint(size_t i) { return joints_[i]; }
    const UnifiedJoint& joint(size_t i) const { return joints_[i]; }
    std::vector<UnifiedJoint>& joints() { return joints_; }

    /// Thread-safe lock
    std::mutex& mutex() { return mutex_; }

    /// Flush CAN receive buffer
    void clearReceiveBuffer();

private:
    void log(int level, const std::string& msg);

    // ── Internal RMD operations ────────────────────────────────
    bool readRmdMotor(size_t index);
    bool writeRmdMotor(size_t index);

    // ── Internal Robstride operations ──────────────────────────
    bool readRobstrideMotor(size_t index);
    bool writeRobstrideMotor(size_t index);

    /**
     * Send extended CAN frame and receive response.
     * @return true if response received within timeout
     */
    bool rsSendAndRecv(uint32_t tx_id, const uint8_t* tx_data,
                       uint32_t* rx_id = nullptr, uint8_t* rx_data = nullptr,
                       uint8_t* rx_len = nullptr, int timeout_ms = 500);

    USBCANDevice can_;
    IntegratedDriverConfig config_;
    std::vector<UnifiedJoint> joints_;
    uint32_t can_channel_ = 0;

    std::mutex mutex_;
    IntegratedLogCallback log_cb_;
};

}  // namespace arm_sdk

#endif  // ARM_SDK__INTEGRATED_DRIVER_HPP_
