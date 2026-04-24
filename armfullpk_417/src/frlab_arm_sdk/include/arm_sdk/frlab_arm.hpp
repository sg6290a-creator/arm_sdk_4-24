/**
 * ============================================================================
 * FrlabArm — 6DOF Manipulator SDK (Robstride03 x2 + MyActuator X4-36 x4)
 * ============================================================================
 *
 * Joint layout:
 *   joint[0] = Joint 1  — Robstride03 ID=1  (Shoulder)
 *   joint[1] = Joint 2  — Robstride03 ID=2  (Upper arm)
 *   joint[2] = Joint 3  — X4-36      ID=1  (Elbow)
 *   joint[3] = Joint 4  — X4-36      ID=2
 *   joint[4] = Joint 5  — X4-36      ID=3
 *   joint[5] = Joint 6  — X4-36      ID=4  (Wrist)
 *
 * 사용법:
 *   FrlabArm arm;
 *   arm.init("can0");  // CANable v2.0: sudo slcand -o -c -s8 /dev/ttyACM0 can0 && sudo ip link set up can0
 *
 *   ArmState fb;
 *   arm.read(fb);                          // 피드백만 읽기
 *   arm.write(pos_cmd, vel_cmd);            // 커맨드만 보내기
 *   arm.step(pos_cmd, vel_cmd, fb);         // 커맨드 → 피드백 한 번에
 *
 * ============================================================================
 */

#ifndef ARM_SDK__FRLAB_ARM_HPP_
#define ARM_SDK__FRLAB_ARM_HPP_

#include "arm_sdk/integrated_driver.hpp"

#include <array>
#include <string>
#include <cstdint>

namespace arm_sdk
{

// ════════════════════════════════════════════════════════════════
//  Hardware constants
// ════════════════════════════════════════════════════════════════

constexpr int ARM_DOF = 6;

/// Robstride03 position mode speed limit (rad/s) — tune as needed
constexpr float RS03_DEFAULT_SPEED_LIMIT = 5.0f;

/// MyActuator X4-36 torque constant (Nm/A) — verify with datasheet
constexpr double X4_36_TORQUE_CONSTANT = 0.32;

/// RMD default velocity in position mode (deg/s)
constexpr double X4_36_DEFAULT_VEL_DPS = 50.0;

// ════════════════════════════════════════════════════════════════
//  ArmState — 6DOF feedback
// ════════════════════════════════════════════════════════════════

struct ArmState {
    std::array<double, ARM_DOF> position;     // rad, joint 1..6
    std::array<double, ARM_DOF> velocity;     // rad/s
    std::array<double, ARM_DOF> effort;       // Nm
    std::array<double, ARM_DOF> temperature;  // °C
    bool valid = false;

    ArmState() {
        position.fill(0.0);
        velocity.fill(0.0);
        effort.fill(0.0);
        temperature.fill(0.0);
    }
};

// ════════════════════════════════════════════════════════════════
//  FrlabArm
// ════════════════════════════════════════════════════════════════

class FrlabArm
{
public:
    FrlabArm();
    ~FrlabArm();

    // Non-copyable
    FrlabArm(const FrlabArm&) = delete;
    FrlabArm& operator=(const FrlabArm&) = delete;

    // ── Lifecycle ──────────────────────────────────────────────

    /**
     * Initialize: open SocketCAN, test all 6 motors, enter position mode.
     * @param can_if  SocketCAN interface name (e.g. "can0")
     * @return true on success
     */
    bool init(
        const std::string& can_if = "can0",
        int read_deadline_ms = 3,
        int read_poll_timeout_ms = 1);

    /**
     * Shutdown: disable Robstride motors, close SocketCAN.
     */
    void shutdown();

    // ── Main interface ─────────────────────────────────────────

    /**
     * Read all 6 joint states.
     * @param[out] state  current position/velocity/effort/temperature
     * @return true if all joints responded
     */
    bool read(ArmState& state);

    /** Read last known joint states without sending CAN requests. */
    void readCached(ArmState& state);

    /**
     * Send position + velocity commands to all 6 joints.
     * @param pos_cmd  desired positions (rad), joint 1..6
     * @param vel_cmd  desired velocities (rad/s), joint 1..6
     *                 - Robstride: velocity ignored in position mode (use setSpeedLimit)
     *                 - RMD: used as max velocity for position profile
     */
    bool write(const std::array<double, ARM_DOF>& pos_cmd,
               const std::array<double, ARM_DOF>& vel_cmd);

    /**
     * One control step: write commands then read feedback.
     * Intended for t-periodic loops.
     *
     * @param pos_cmd  desired positions (rad)
     * @param vel_cmd  desired velocities (rad/s)
     * @param[out] fb  resulting feedback state
     * @return true if both write and read succeeded
     */
    bool step(const std::array<double, ARM_DOF>& pos_cmd,
              const std::array<double, ARM_DOF>& vel_cmd,
              ArmState& fb);

    // ── Configuration helpers ──────────────────────────────────

    /**
     * Set Robstride03 position mode speed limit.
     * @param joint_idx  0 or 1 (joint 1 or 2)
     * @param speed_rads  speed limit in rad/s
     */
    bool setRobstrideSpeedLimit(size_t joint_idx, float speed_rads);

    /**
     * Set RMD acceleration for a joint.
     * @param joint_idx  2..5 (joint 3..6)
     * @param accel_dps2 acceleration in deg/s²
     */
    bool setRmdAcceleration(size_t joint_idx, uint32_t accel_dps2);

    // ── Direct driver access ───────────────────────────────────

    IntegratedDriver& driver() { return driver_; }

private:
    void buildConfig(
        const std::string& can_if,
        IntegratedDriverConfig& cfg,
        int read_deadline_ms,
        int read_poll_timeout_ms);

    IntegratedDriver driver_;
    bool initialized_ = false;
};

}  // namespace arm_sdk

#endif  // ARM_SDK__FRLAB_ARM_HPP_
