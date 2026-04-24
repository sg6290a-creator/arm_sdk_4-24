/**
 * ============================================================================
 * RMDProtocol — MyActuator RMD 모터 CAN 프로토콜 (순수 C++)
 * ============================================================================
 *
 * CAN 명령 빌드/파싱: 0x9C(상태 읽기), 0xA4(위치 제어), 0x43(가속도 설정)
 * 단위 변환: degree ↔ radian, dps ↔ rad/s
 *
 * ============================================================================
 */

#ifndef ARM_SDK__RMD_PROTOCOL_HPP_
#define ARM_SDK__RMD_PROTOCOL_HPP_

#include <cstdint>
#include <cmath>

namespace arm_sdk
{

/// CAN ID convention for MyActuator RMD motors
constexpr uint32_t RMD_TX_BASE = 0x140;  // Host → Motor: 0x140 + motor_id
constexpr uint32_t RMD_RX_BASE = 0x240;  // Motor → Host: 0x240 + motor_id

/// RMD CAN command bytes
constexpr uint8_t RMD_CMD_READ_STATUS2    = 0x9C;
constexpr uint8_t RMD_CMD_POSITION_CTRL2  = 0xA4;
constexpr uint8_t RMD_CMD_SET_ACCEL       = 0x43;

/// Motor state returned from readMotorState
struct MotorState {
    double position_rad  = 0.0;  // shaft angle in radians
    double velocity_rads = 0.0;  // speed in rad/s
    double effort_nm     = 0.0;  // torque = current × torque_constant
    int8_t temperature   = 0;    // °C
    bool   valid         = false;
};

/// Motor command parameters
struct MotorCommand {
    double position_rad     = 0.0;
    double velocity_rads    = 0.0;
    double default_vel_dps  = 50.0;   // fallback velocity (dps)
    double max_vel_dps      = 360.0;  // velocity limit (dps)
};

class RMDProtocol
{
public:
    // ── Unit Conversion ────────────────────────────────────────

    static inline double degToRad(double deg) { return deg * M_PI / 180.0; }
    static inline double radToDeg(double rad) { return rad * 180.0 / M_PI; }
    static inline double dpsToRadS(double dps) { return dps * M_PI / 180.0; }
    static inline double radSToDps(double rads) { return rads * 180.0 / M_PI; }

    // ── TX ID / RX ID ──────────────────────────────────────────

    static inline uint32_t txId(int motor_id) { return RMD_TX_BASE + motor_id; }
    static inline uint32_t rxId(int motor_id) { return RMD_RX_BASE + motor_id; }

    // ── Command Builders (populate 8-byte CAN data) ────────────

    /**
     * Build 0x9C "Read Motor Status 2" command.
     * @param[out] data  8-byte CAN data buffer
     */
    static void buildReadStatus2(uint8_t data[8]);

    /**
     * Build 0xA4 "Position Closed-Loop Control 2" command.
     * @param[out] data   8-byte CAN data buffer
     * @param cmd         Motor command (position, velocity)
     */
    static void buildPositionCtrl2(uint8_t data[8], const MotorCommand& cmd);

    /**
     * Build 0x43 "Write Acceleration to RAM" command.
     * @param[out] data   8-byte CAN data buffer
     * @param accel_dps2  Acceleration in deg/s²
     */
    static void buildSetAcceleration(uint8_t data[8], uint32_t accel_dps2);

    // ── Response Parser ────────────────────────────────────────

    /**
     * Parse 0x9C motor status 2 response.
     * @param rx_data         8-byte response data
     * @param rx_len          response length (must be >= 8)
     * @param torque_constant torque constant (Nm/A) for effort calculation
     * @return MotorState with valid=true on success
     */
    static MotorState parseStatus2(const uint8_t rx_data[8], uint8_t rx_len,
                                    double torque_constant = 1.0);
};

}  // namespace arm_sdk

#endif  // ARM_SDK__RMD_PROTOCOL_HPP_
