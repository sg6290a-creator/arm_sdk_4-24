/**
 * ============================================================================
 * RMDProtocol — Implementation
 * ============================================================================
 */

#include "arm_sdk/rmd_protocol.hpp"

#include <cstring>
#include <cstdlib>

namespace arm_sdk
{

// ════════════════════════════════════════════════════════════════
//  Command Builders
// ════════════════════════════════════════════════════════════════

void RMDProtocol::buildReadStatus2(uint8_t data[8])
{
    std::memset(data, 0, 8);
    data[0] = RMD_CMD_READ_STATUS2;  // 0x9C
}

void RMDProtocol::buildPositionCtrl2(uint8_t data[8], const MotorCommand& cmd)
{
    double position_deg = radToDeg(cmd.position_rad);
    double velocity_dps = std::abs(radSToDps(cmd.velocity_rads));

    // Apply velocity limits
    if (velocity_dps < 5.0) velocity_dps = cmd.default_vel_dps;
    if (velocity_dps > cmd.max_vel_dps) velocity_dps = cmd.max_vel_dps;

    int32_t  angle_ctrl = static_cast<int32_t>(position_deg * 100);  // 0.01 deg units
    uint16_t speed_ctrl = static_cast<uint16_t>(velocity_dps);

    data[0] = RMD_CMD_POSITION_CTRL2;  // 0xA4
    data[1] = 0x00;                     // spin direction (auto)
    data[2] = speed_ctrl & 0xFF;
    data[3] = (speed_ctrl >> 8) & 0xFF;
    data[4] = angle_ctrl & 0xFF;
    data[5] = (angle_ctrl >> 8) & 0xFF;
    data[6] = (angle_ctrl >> 16) & 0xFF;
    data[7] = (angle_ctrl >> 24) & 0xFF;
}

void RMDProtocol::buildSetAcceleration(uint8_t data[8], uint32_t accel_dps2)
{
    data[0] = RMD_CMD_SET_ACCEL;  // 0x43
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = accel_dps2 & 0xFF;
    data[5] = (accel_dps2 >> 8) & 0xFF;
    data[6] = (accel_dps2 >> 16) & 0xFF;
    data[7] = (accel_dps2 >> 24) & 0xFF;
}

// ════════════════════════════════════════════════════════════════
//  Response Parser
// ════════════════════════════════════════════════════════════════

MotorState RMDProtocol::parseStatus2(const uint8_t rx_data[8], uint8_t rx_len,
                                      double torque_constant)
{
    MotorState state;

    // Validate: must be 0x9C response with 8 bytes
    if (rx_len < 8 || rx_data[0] != RMD_CMD_READ_STATUS2) {
        return state;  // valid = false
    }

    // Byte 1: temperature (°C)
    state.temperature = static_cast<int8_t>(rx_data[1]);

    // Byte 2-3: torque current (int16, 0.01A)
    int16_t current_raw = static_cast<int16_t>((rx_data[3] << 8) | rx_data[2]);

    // Byte 4-5: speed (int16, 1 dps)
    int16_t speed_raw = static_cast<int16_t>((rx_data[5] << 8) | rx_data[4]);

    // Byte 6-7: shaft angle (int16, degrees)
    int16_t shaft_angle = static_cast<int16_t>((rx_data[7] << 8) | rx_data[6]);

    state.position_rad  = degToRad(static_cast<double>(shaft_angle));
    state.velocity_rads = dpsToRadS(static_cast<double>(speed_raw));
    state.effort_nm     = current_raw * 0.01 * torque_constant;
    state.valid         = true;

    return state;
}

}  // namespace arm_sdk
