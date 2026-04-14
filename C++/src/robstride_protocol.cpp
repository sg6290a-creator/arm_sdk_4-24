/**
 * ============================================================================
 * RobstrideProtocol — Implementation
 * ============================================================================
 */

#include "arm_sdk/robstride_protocol.hpp"

namespace arm_sdk
{

// ════════════════════════════════════════════════════════════════
//  Arbitration ID
// ════════════════════════════════════════════════════════════════

uint32_t RobstrideProtocol::makeArbId(uint8_t msg_type, uint16_t data1, uint8_t motor_id)
{
    return (static_cast<uint32_t>(msg_type) << 24) |
           (static_cast<uint32_t>(data1) << 8) |
           motor_id;
}

uint8_t RobstrideProtocol::getMsgType(uint32_t arb_id)
{
    return static_cast<uint8_t>((arb_id >> 24) & 0x1F);
}

// ════════════════════════════════════════════════════════════════
//  Value Mapping
// ════════════════════════════════════════════════════════════════

float RobstrideProtocol::uint16ToFloat(uint16_t x, float x_min, float x_max)
{
    return (x_max - x_min) * static_cast<float>(x) / 65535.0f + x_min;
}

uint16_t RobstrideProtocol::floatToUint16(float x, float x_min, float x_max)
{
    if (x > x_max) x = x_max;
    if (x < x_min) x = x_min;
    return static_cast<uint16_t>((x - x_min) / (x_max - x_min) * 65535.0f);
}

// ════════════════════════════════════════════════════════════════
//  Feedback Parsing
// ════════════════════════════════════════════════════════════════

RobstrideState RobstrideProtocol::parseFeedback(uint32_t arb_id, const uint8_t data[8])
{
    RobstrideState state;

    state.motor_id   = static_cast<uint8_t>((arb_id >> 8) & 0xFF);
    state.error_bits = static_cast<uint8_t>((arb_id >> 16) & 0x3F);
    state.mode       = static_cast<uint8_t>((arb_id >> 22) & 0x03);

    uint16_t angle_raw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    uint16_t vel_raw   = (static_cast<uint16_t>(data[2]) << 8) | data[3];
    uint16_t torq_raw  = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    uint16_t temp_raw  = (static_cast<uint16_t>(data[6]) << 8) | data[7];

    state.position_rad  = uint16ToFloat(angle_raw, RS_ANGLE_MIN, RS_ANGLE_MAX);
    state.velocity_rads = uint16ToFloat(vel_raw, RS_VEL_MIN, RS_VEL_MAX);
    state.torque_nm     = uint16ToFloat(torq_raw, RS_TORQUE_MIN, RS_TORQUE_MAX);
    state.temperature   = static_cast<float>(temp_raw) / 10.0f;
    state.valid         = true;

    return state;
}

// ════════════════════════════════════════════════════════════════
//  Command Builders
// ════════════════════════════════════════════════════════════════

void RobstrideProtocol::buildEnable(uint32_t& arb_id, uint8_t data[8],
                                    uint8_t motor_id, uint8_t host_id)
{
    std::memset(data, 0, 8);
    arb_id = makeArbId(RS_MSG_ENABLE, host_id, motor_id);
}

void RobstrideProtocol::buildDisable(uint32_t& arb_id, uint8_t data[8],
                                     uint8_t motor_id, uint8_t clear_error,
                                     uint8_t host_id)
{
    std::memset(data, 0, 8);
    data[0] = clear_error;
    arb_id = makeArbId(RS_MSG_DISABLE, host_id, motor_id);
}

void RobstrideProtocol::buildSetZero(uint32_t& arb_id, uint8_t data[8],
                                     uint8_t motor_id, uint8_t host_id)
{
    std::memset(data, 0, 8);
    data[0] = 1;  // save flag
    arb_id = makeArbId(RS_MSG_SET_ZERO, host_id, motor_id);
}

void RobstrideProtocol::buildReadParam(uint32_t& arb_id, uint8_t data[8],
                                       uint8_t motor_id, uint16_t param_id,
                                       uint8_t host_id)
{
    std::memset(data, 0, 8);
    data[0] = param_id & 0xFF;
    data[1] = (param_id >> 8) & 0xFF;
    arb_id = makeArbId(RS_MSG_READ_PARAM, host_id, motor_id);
}

void RobstrideProtocol::buildWriteParam(uint32_t& arb_id, uint8_t data[8],
                                        uint8_t motor_id, uint16_t param_id,
                                        float value, uint8_t host_id)
{
    std::memset(data, 0, 8);
    data[0] = param_id & 0xFF;
    data[1] = (param_id >> 8) & 0xFF;

    if (param_id == RS_PARAM_RUN_MODE)
    {
        // Run mode is written as uint8_t
        data[4] = static_cast<uint8_t>(value);
    }
    else
    {
        std::memcpy(&data[4], &value, 4);
    }

    arb_id = makeArbId(RS_MSG_WRITE_PARAM, host_id, motor_id);
}

// ════════════════════════════════════════════════════════════════
//  Parameter Response Parsing
// ════════════════════════════════════════════════════════════════

bool RobstrideProtocol::parseParamResponse(const uint8_t data[8],
                                           uint16_t expected_param,
                                           float& value)
{
    uint16_t resp_param = data[0] | (static_cast<uint16_t>(data[1]) << 8);

    if (resp_param != expected_param)
        return false;

    if (expected_param == RS_PARAM_RUN_MODE)
    {
        value = static_cast<float>(data[4]);
    }
    else
    {
        std::memcpy(&value, &data[4], 4);
    }
    return true;
}

// ════════════════════════════════════════════════════════════════
//  String Helpers
// ════════════════════════════════════════════════════════════════

const char* RobstrideProtocol::modeString(uint8_t mode)
{
    switch (mode) {
        case 0: return "Reset";
        case 1: return "Calibration";
        case 2: return "Run";
        default: return "Unknown";
    }
}

const char* RobstrideProtocol::runModeString(uint8_t run_mode)
{
    switch (run_mode) {
        case RS_MODE_MOTION:   return "Motion(MIT)";
        case RS_MODE_POSITION: return "Position";
        case RS_MODE_SPEED:    return "Speed";
        case RS_MODE_CURRENT:  return "Current";
        default: return "Unknown";
    }
}

}  // namespace arm_sdk
