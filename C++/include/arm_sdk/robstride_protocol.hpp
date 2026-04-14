/**
 * ============================================================================
 * RobstrideProtocol — Robstride 모터 CAN 프로토콜 (순수 C++)
 * ============================================================================
 *
 * Extended CAN (29-bit) 프로토콜.
 * Arbitration ID = [msg_type:5][data1:16][motor_id:8]
 *
 * Enable/Disable, 파라미터 읽기/쓰기, 위치/속도/전류 모드 지원.
 * RMDProtocol과 대칭 구조로 IntegratedDriver에서 함께 사용.
 *
 * ============================================================================
 */

#ifndef ARM_SDK__ROBSTRIDE_PROTOCOL_HPP_
#define ARM_SDK__ROBSTRIDE_PROTOCOL_HPP_

#include <cstdint>
#include <cstring>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace arm_sdk
{

// ════════════════════════════════════════════════════════════════
//  Constants
// ════════════════════════════════════════════════════════════════

/// CAN message types (bits 28-24 of extended arbitration ID)
constexpr uint8_t RS_MSG_GET_ID      = 0x00;
constexpr uint8_t RS_MSG_CONTROL     = 0x01;  // Motion control (MIT mode)
constexpr uint8_t RS_MSG_FEEDBACK    = 0x02;  // Motor feedback
constexpr uint8_t RS_MSG_ENABLE      = 0x03;
constexpr uint8_t RS_MSG_DISABLE     = 0x04;
constexpr uint8_t RS_MSG_SET_ZERO    = 0x06;
constexpr uint8_t RS_MSG_SET_ID      = 0x07;
constexpr uint8_t RS_MSG_READ_PARAM  = 0x11;
constexpr uint8_t RS_MSG_WRITE_PARAM = 0x12;

/// Parameter indices
constexpr uint16_t RS_PARAM_RUN_MODE      = 0x7005;
constexpr uint16_t RS_PARAM_SPD_REF       = 0x700A;
constexpr uint16_t RS_PARAM_LIMIT_TORQUE  = 0x700B;
constexpr uint16_t RS_PARAM_LOC_REF       = 0x7016;
constexpr uint16_t RS_PARAM_LIMIT_SPD     = 0x7017;
constexpr uint16_t RS_PARAM_LIMIT_CUR     = 0x7018;
constexpr uint16_t RS_PARAM_MECH_POS      = 0x7019;
constexpr uint16_t RS_PARAM_MECH_VEL      = 0x701B;
constexpr uint16_t RS_PARAM_VBUS          = 0x701C;
constexpr uint16_t RS_PARAM_LOC_KP        = 0x701E;
constexpr uint16_t RS_PARAM_SPD_KP        = 0x701F;
constexpr uint16_t RS_PARAM_SPD_KI        = 0x7020;

/// Run modes
constexpr uint8_t RS_MODE_MOTION   = 0;  // MIT mode
constexpr uint8_t RS_MODE_POSITION = 1;
constexpr uint8_t RS_MODE_SPEED    = 2;
constexpr uint8_t RS_MODE_CURRENT  = 3;

/// Feedback value ranges (RS01 series, model=1)
constexpr float RS_ANGLE_MIN = -4.0f * static_cast<float>(M_PI);
constexpr float RS_ANGLE_MAX =  4.0f * static_cast<float>(M_PI);
constexpr float RS_VEL_MIN   = -44.0f;
constexpr float RS_VEL_MAX   =  44.0f;
constexpr float RS_TORQUE_MIN = -17.0f;
constexpr float RS_TORQUE_MAX =  17.0f;

/// Default host CAN ID
constexpr uint8_t RS_HOST_CAN_ID = 0x00;

// ════════════════════════════════════════════════════════════════
//  Structs
// ════════════════════════════════════════════════════════════════

/// Motor state parsed from feedback response
struct RobstrideState {
    float   position_rad  = 0.0f;   // mechanical angle (rad)
    float   velocity_rads = 0.0f;   // angular velocity (rad/s)
    float   torque_nm     = 0.0f;   // torque (Nm)
    float   temperature   = 0.0f;   // °C (raw / 10)
    uint8_t motor_id      = 0;
    uint8_t mode          = 0;      // 0=Reset, 1=Calibration, 2=Run
    uint8_t error_bits    = 0;
    bool    valid         = false;
};

// ════════════════════════════════════════════════════════════════
//  RobstrideProtocol
// ════════════════════════════════════════════════════════════════

class RobstrideProtocol
{
public:
    // ── Arbitration ID ─────────────────────────────────────

    /**
     * Build 29-bit extended CAN arbitration ID.
     * Layout: [msg_type:5 bits][data1:16 bits][motor_id:8 bits]
     */
    static uint32_t makeArbId(uint8_t msg_type, uint16_t data1, uint8_t motor_id);

    /**
     * Parse msg_type from arbitration ID.
     */
    static uint8_t getMsgType(uint32_t arb_id);

    // ── Feedback Parsing ───────────────────────────────────

    /**
     * Parse feedback response from CAN arbitration ID + 8-byte data.
     * @param arb_id  29-bit extended CAN ID
     * @param data    8-byte response data
     * @return RobstrideState with valid=true on success
     */
    static RobstrideState parseFeedback(uint32_t arb_id, const uint8_t data[8]);

    // ── Command Builders ───────────────────────────────────
    // Each sets arb_id (output) and data[8] (output) for CAN transmission.

    /**
     * Build Enable motor command.
     */
    static void buildEnable(uint32_t& arb_id, uint8_t data[8],
                            uint8_t motor_id, uint8_t host_id = RS_HOST_CAN_ID);

    /**
     * Build Disable motor command.
     * @param clear_error  set to 1 to clear fault bits
     */
    static void buildDisable(uint32_t& arb_id, uint8_t data[8],
                             uint8_t motor_id, uint8_t clear_error = 0,
                             uint8_t host_id = RS_HOST_CAN_ID);

    /**
     * Build Set Zero Position command.
     * Motor must be disabled first.
     */
    static void buildSetZero(uint32_t& arb_id, uint8_t data[8],
                             uint8_t motor_id, uint8_t host_id = RS_HOST_CAN_ID);

    /**
     * Build Read Parameter command.
     */
    static void buildReadParam(uint32_t& arb_id, uint8_t data[8],
                               uint8_t motor_id, uint16_t param_id,
                               uint8_t host_id = RS_HOST_CAN_ID);

    /**
     * Build Write Parameter command.
     * For PARAM_RUN_MODE, value is cast to uint8_t.
     * For others, value is written as IEEE 754 float.
     */
    static void buildWriteParam(uint32_t& arb_id, uint8_t data[8],
                                uint8_t motor_id, uint16_t param_id, float value,
                                uint8_t host_id = RS_HOST_CAN_ID);

    // ── Parameter Response Parsing ─────────────────────────

    /**
     * Parse read-parameter response.
     * @param data           8-byte response data
     * @param expected_param expected parameter ID (for validation)
     * @param[out] value     parsed value
     * @return true if param_id matched
     */
    static bool parseParamResponse(const uint8_t data[8], uint16_t expected_param,
                                   float& value);

    // ── Value Mapping ──────────────────────────────────────

    static float uint16ToFloat(uint16_t x, float x_min, float x_max);
    static uint16_t floatToUint16(float x, float x_min, float x_max);

    // ── Mode String ────────────────────────────────────────

    static const char* modeString(uint8_t mode);
    static const char* runModeString(uint8_t run_mode);
};

}  // namespace arm_sdk

#endif  // ARM_SDK__ROBSTRIDE_PROTOCOL_HPP_
