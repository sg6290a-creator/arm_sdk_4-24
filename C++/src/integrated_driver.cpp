/**
 * ============================================================================
 * IntegratedDriver — Implementation
 * ============================================================================
 *
 * RMD + Robstride 모터를 1개의 USBCAN 디바이스에서 통합 제어.
 * CAN 프레임 구분: extern_flag (0=RMD, 1=Robstride)
 *
 * ============================================================================
 */

#include "arm_sdk/integrated_driver.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

namespace arm_sdk
{

IntegratedDriver::IntegratedDriver() = default;

IntegratedDriver::~IntegratedDriver()
{
    cleanup();
}

void IntegratedDriver::setLogCallback(IntegratedLogCallback cb)
{
    log_cb_ = std::move(cb);
}

void IntegratedDriver::log(int level, const std::string& msg)
{
    if (log_cb_) {
        log_cb_(level, msg);
    } else {
        const char* prefix = (level == LOG_ERROR) ? "[ERROR]" :
                             (level == LOG_WARN)  ? "[WARN]"  : "[INFO]";
        fprintf(stderr, "[IntegratedDriver] %s %s\n", prefix, msg.c_str());
    }
}

// ════════════════════════════════════════════════════════════════
//  Lifecycle
// ════════════════════════════════════════════════════════════════

bool IntegratedDriver::configure(const IntegratedDriverConfig& config)
{
    config_ = config;
    can_channel_ = config.can_channel;

    // ── Step 1: Load USBCAN library ────────────────────────
    log(LOG_INFO, "[1/4] Loading " + config.lib_path + "...");
    if (!can_.loadLibrary(config.lib_path)) {
        log(LOG_ERROR, "Failed to load USBCAN library: " + config.lib_path);
        return false;
    }

    // ── Step 2: Open device ────────────────────────────────
    log(LOG_INFO, "[2/4] Opening USBCAN (type=" +
        std::to_string(config.device_type) + ", idx=" +
        std::to_string(config.device_index) + ")...");
    if (!can_.openDevice(config.device_type, config.device_index)) {
        log(LOG_ERROR, "Failed to open USBCAN device");
        can_.unloadLibrary();
        return false;
    }

    // ── Step 3: Init CAN channel ───────────────────────────
    uint16_t timing = USBCANDevice::baudrateToTiming(config.baudrate);
    char timing_str[16];
    snprintf(timing_str, sizeof(timing_str), "0x%04X", timing);
    log(LOG_INFO, "[3/4] Init CAN ch" + std::to_string(config.can_channel) +
        " (timing=" + timing_str + ", " + std::to_string(config.baudrate/1000) + "kbps)...");

    if (!can_.initChannel(config.can_channel, timing) ||
        !can_.startChannel(config.can_channel))
    {
        log(LOG_ERROR, "Failed to init/start CAN channel");
        can_.closeDevice();
        can_.unloadLibrary();
        return false;
    }

    // ── Step 4: Initialize joints and test communication ───
    joints_.resize(config.joints.size());
    for (size_t i = 0; i < config.joints.size(); ++i) {
        const auto& def = config.joints[i];
        auto& j = joints_[i];
        j.name               = def.name;
        j.motor_type         = def.motor_type;
        j.motor_id           = def.motor_id;
        j.torque_constant    = def.torque_constant;
        j.robstride_max_speed = def.max_speed_rads;
    }

    // Flush any pending frames
    clearReceiveBuffer();

    // Count motor types
    int rmd_count = 0, rs_count = 0;
    for (const auto& j : joints_) {
        if (j.motor_type == MotorType::RMD) rmd_count++;
        else rs_count++;
    }

    log(LOG_INFO, "[4/4] Testing " + std::to_string(joints_.size()) +
        " motors (" + std::to_string(rmd_count) + " RMD + " +
        std::to_string(rs_count) + " Robstride)...");

    for (size_t i = 0; i < joints_.size(); ++i) {
        auto& j = joints_[i];
        bool ok = false;

        if (j.motor_type == MotorType::RMD) {
            ok = readRmdMotor(i);
        } else {
            // Robstride: try enable → get feedback → disable
            ok = enableRobstride(i);
            if (ok) {
                j.enabled = true;
                disableRobstride(i);
                j.enabled = false;
            }
        }

        if (ok) {
            char buf[128];
            snprintf(buf, sizeof(buf),
                     "  [OK] '%s' (%s, ID=%d): pos=%.2f°",
                     j.name.c_str(), motorTypeString(j.motor_type),
                     j.motor_id,
                     j.position_rad * 180.0 / M_PI);
            log(LOG_INFO, buf);
        } else {
            log(LOG_ERROR, "  [FAIL] '" + j.name + "' (" +
                motorTypeString(j.motor_type) + ", ID=" +
                std::to_string(j.motor_id) + ") — no response");
            can_.closeDevice();
            can_.unloadLibrary();
            return false;
        }
    }

    log(LOG_INFO, "Configuration complete: " + std::to_string(joints_.size()) +
        " motors on CAN ch" + std::to_string(can_channel_));
    return true;
}

bool IntegratedDriver::activate()
{
    log(LOG_INFO, "Activating motors...");

    for (size_t i = 0; i < joints_.size(); ++i) {
        auto& j = joints_[i];

        if (j.motor_type == MotorType::RMD)
        {
            // Set acceleration
            setRmdAcceleration(i, config_.rmd_acceleration);

            // Read current position
            if (readRmdMotor(i)) {
                j.position_command = j.position_rad;
                j.enabled = true;
                char buf[128];
                snprintf(buf, sizeof(buf), "  RMD '%s' (ID=%d): %.2f° — ready",
                         j.name.c_str(), j.motor_id,
                         j.position_rad * 180.0 / M_PI);
                log(LOG_INFO, buf);
            } else {
                log(LOG_WARN, "  RMD '" + j.name + "' read failed during activate");
            }
        }
        else  // ROBSTRIDE
        {
            // Disable → set position mode → set speed limit → enable
            disableRobstride(i);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            writeRobstrideParam(i, RS_PARAM_RUN_MODE, RS_MODE_POSITION);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            writeRobstrideParam(i, RS_PARAM_LIMIT_SPD, j.robstride_max_speed);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            if (enableRobstride(i)) {
                j.enabled = true;

                // Read mechanical position
                float mech_pos = 0;
                if (readRobstrideParam(i, RS_PARAM_MECH_POS, mech_pos)) {
                    j.position_rad = static_cast<double>(mech_pos);
                    j.position_command = j.position_rad;
                }

                char buf[128];
                snprintf(buf, sizeof(buf),
                         "  Robstride '%s' (ID=%d): %.2f° — position mode, ready",
                         j.name.c_str(), j.motor_id,
                         j.position_rad * 180.0 / M_PI);
                log(LOG_INFO, buf);
            } else {
                log(LOG_WARN, "  Robstride '" + j.name + "' enable failed");
            }
        }
    }

    log(LOG_INFO, "Activation complete");
    return true;
}

void IntegratedDriver::deactivate()
{
    log(LOG_INFO, "Deactivating motors...");

    for (size_t i = 0; i < joints_.size(); ++i) {
        auto& j = joints_[i];
        if (j.motor_type == MotorType::ROBSTRIDE && j.enabled) {
            disableRobstride(i);
            j.enabled = false;
            log(LOG_INFO, "  Robstride '" + j.name + "' disabled");
        }
        // RMD: no explicit disable needed (just stop sending commands)
        j.enabled = false;
    }

    log(LOG_INFO, "Deactivation complete");
}

void IntegratedDriver::cleanup()
{
    deactivate();
    can_.closeDevice();
    can_.unloadLibrary();
}

// ════════════════════════════════════════════════════════════════
//  Communication — Read
// ════════════════════════════════════════════════════════════════

bool IntegratedDriver::readAll()
{
    bool any_ok = false;
    for (size_t i = 0; i < joints_.size(); ++i) {
        if (readMotor(i))
            any_ok = true;
    }
    return any_ok;
}

bool IntegratedDriver::readMotor(size_t index)
{
    if (index >= joints_.size()) return false;

    if (joints_[index].motor_type == MotorType::RMD)
        return readRmdMotor(index);
    else
        return readRobstrideMotor(index);
}

// ════════════════════════════════════════════════════════════════
//  Communication — Write
// ════════════════════════════════════════════════════════════════

void IntegratedDriver::writeAll()
{
    for (size_t i = 0; i < joints_.size(); ++i) {
        writeMotor(i);
    }
}

bool IntegratedDriver::writeMotor(size_t index)
{
    if (index >= joints_.size()) return false;

    if (joints_[index].motor_type == MotorType::RMD)
        return writeRmdMotor(index);
    else
        return writeRobstrideMotor(index);
}

// ════════════════════════════════════════════════════════════════
//  Internal — RMD
// ════════════════════════════════════════════════════════════════

bool IntegratedDriver::readRmdMotor(size_t index)
{
    auto& j = joints_[index];
    uint8_t cmd[8];
    RMDProtocol::buildReadStatus2(cmd);
    uint32_t tx_id = RMDProtocol::txId(j.motor_id);
    uint32_t expected_rx = RMDProtocol::rxId(j.motor_id);

    if (!can_.sendFrame(can_channel_, tx_id, cmd, 8, 0))  // extern_flag=0 (standard)
        return false;

    // Poll for response (standard CAN only)
    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;

    for (int attempt = 0; attempt < 20; ++attempt) {
        uint8_t ext = 0;
        if (can_.receiveFrame(can_channel_, rx_id, rx_data, rx_len, 2, &ext)) {
            if (ext == 0 && rx_id == expected_rx) {
                MotorState state = RMDProtocol::parseStatus2(rx_data, rx_len, j.torque_constant);
                if (state.valid) {
                    j.position_rad  = state.position_rad;
                    j.velocity_rads = state.velocity_rads;
                    j.effort_nm     = state.effort_nm;
                    j.temperature   = state.temperature;
                    return true;
                }
            }
            // Ignore extended frames (from Robstride broadcast etc.)
        }
    }
    return false;
}

bool IntegratedDriver::writeRmdMotor(size_t index)
{
    auto& j = joints_[index];
    uint8_t cmd[8];
    MotorCommand mc;
    mc.position_rad    = j.position_command;
    mc.velocity_rads   = j.velocity_command;
    mc.default_vel_dps = config_.rmd_default_vel_dps;
    mc.max_vel_dps     = config_.rmd_max_vel_dps;

    RMDProtocol::buildPositionCtrl2(cmd, mc);
    return can_.sendFrame(can_channel_, RMDProtocol::txId(j.motor_id), cmd, 8, 0);
}

bool IntegratedDriver::setRmdAcceleration(size_t index, uint32_t accel_dps2)
{
    if (index >= joints_.size() || joints_[index].motor_type != MotorType::RMD)
        return false;

    auto& j = joints_[index];
    uint8_t cmd[8];
    RMDProtocol::buildSetAcceleration(cmd, accel_dps2);

    if (!can_.sendFrame(can_channel_, RMDProtocol::txId(j.motor_id), cmd, 8, 0))
        return false;

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    return true;
}

// ════════════════════════════════════════════════════════════════
//  Internal — Robstride
// ════════════════════════════════════════════════════════════════

bool IntegratedDriver::rsSendAndRecv(uint32_t tx_id, const uint8_t* tx_data,
                                     uint32_t* rx_id, uint8_t* rx_data,
                                     uint8_t* rx_len, int timeout_ms)
{
    if (!can_.sendFrame(can_channel_, tx_id, tx_data, 8, 1))  // extern_flag=1
        return false;

    uint32_t id;
    uint8_t data[8], len;

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        uint8_t ext = 0;
        if (can_.receiveFrame(can_channel_, id, data, len, 2, &ext)) {
            if (ext == 1) {
                // Extended frame → Robstride response
                if (rx_id)   *rx_id = id;
                if (rx_data) std::memcpy(rx_data, data, 8);
                if (rx_len)  *rx_len = len;
                return true;
            }
            // Standard frame → RMD response (ignore here)
        }
    }
    return false;
}

bool IntegratedDriver::readRobstrideMotor(size_t index)
{
    auto& j = joints_[index];

    // Enable 명령을 보내면 피드백 프레임으로 position/velocity/torque/temp 전부 응답
    // (이미 Enable 상태에서 다시 보내도 피드백만 오고 상태 변화 없음)
    uint32_t arb_id;
    uint8_t data[8];
    RobstrideProtocol::buildEnable(arb_id, data, static_cast<uint8_t>(j.motor_id));

    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;

    if (!rsSendAndRecv(arb_id, data, &rx_id, rx_data, &rx_len))
        return false;

    RobstrideState state = RobstrideProtocol::parseFeedback(rx_id, rx_data);
    if (state.valid) {
        j.position_rad  = static_cast<double>(state.position_rad);
        j.velocity_rads = static_cast<double>(state.velocity_rads);
        j.effort_nm     = static_cast<double>(state.torque_nm);
        j.temperature   = static_cast<double>(state.temperature);
        j.rs_mode       = state.mode;
        j.rs_error_bits = state.error_bits;
        return true;
    }

    return false;
}

bool IntegratedDriver::writeRobstrideMotor(size_t index)
{
    auto& j = joints_[index];

    // Write position reference
    float target = static_cast<float>(j.position_command);
    return writeRobstrideParam(index, RS_PARAM_LOC_REF, target);
}

bool IntegratedDriver::enableRobstride(size_t index)
{
    if (index >= joints_.size() || joints_[index].motor_type != MotorType::ROBSTRIDE)
        return false;

    auto& j = joints_[index];
    uint32_t arb_id;
    uint8_t data[8];
    RobstrideProtocol::buildEnable(arb_id, data, static_cast<uint8_t>(j.motor_id));

    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;

    if (!rsSendAndRecv(arb_id, data, &rx_id, rx_data, &rx_len))
        return false;

    RobstrideState state = RobstrideProtocol::parseFeedback(rx_id, rx_data);
    if (state.valid) {
        j.position_rad  = static_cast<double>(state.position_rad);
        j.velocity_rads = static_cast<double>(state.velocity_rads);
        j.effort_nm     = static_cast<double>(state.torque_nm);
        j.temperature   = static_cast<double>(state.temperature);
        j.rs_mode       = state.mode;
        j.rs_error_bits = state.error_bits;
        j.enabled       = true;
    }

    return state.valid;
}

bool IntegratedDriver::disableRobstride(size_t index)
{
    if (index >= joints_.size() || joints_[index].motor_type != MotorType::ROBSTRIDE)
        return false;

    auto& j = joints_[index];
    uint32_t arb_id;
    uint8_t data[8];
    RobstrideProtocol::buildDisable(arb_id, data, static_cast<uint8_t>(j.motor_id));

    bool ok = can_.sendFrame(can_channel_, arb_id, data, 8, 1);
    if (ok) j.enabled = false;
    return ok;
}

bool IntegratedDriver::setRobstrideMode(size_t index, uint8_t mode)
{
    if (index >= joints_.size() || joints_[index].motor_type != MotorType::ROBSTRIDE)
        return false;

    // Disable → change mode → enable
    disableRobstride(index);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    writeRobstrideParam(index, RS_PARAM_RUN_MODE, static_cast<float>(mode));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    return enableRobstride(index);
}

bool IntegratedDriver::readRobstrideParam(size_t index, uint16_t param_id, float& value)
{
    if (index >= joints_.size() || joints_[index].motor_type != MotorType::ROBSTRIDE)
        return false;

    auto& j = joints_[index];
    uint32_t arb_id;
    uint8_t data[8];
    RobstrideProtocol::buildReadParam(arb_id, data, static_cast<uint8_t>(j.motor_id), param_id);

    if (!can_.sendFrame(can_channel_, arb_id, data, 8, 1))
        return false;

    // Robstride sends feedback frame first, then param response.
    // Keep reading until we find the matching param response.
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);

    while (std::chrono::steady_clock::now() < deadline) {
        uint32_t rx_id;
        uint8_t rx_data[8], rx_len;
        uint8_t ext = 0;

        if (can_.receiveFrame(can_channel_, rx_id, rx_data, rx_len, 5, &ext)) {
            if (ext != 1) continue;  // skip standard CAN frames

            // Skip feedback frames (msg_type = 0x02)
            uint8_t msg_type = RobstrideProtocol::getMsgType(rx_id);
            if (msg_type == RS_MSG_FEEDBACK) continue;

            // Try to parse as param response
            if (RobstrideProtocol::parseParamResponse(rx_data, param_id, value))
                return true;
            // Wrong param_id — might be stale, keep reading
        }
    }
    return false;
}

bool IntegratedDriver::writeRobstrideParam(size_t index, uint16_t param_id, float value)
{
    if (index >= joints_.size() || joints_[index].motor_type != MotorType::ROBSTRIDE)
        return false;

    auto& j = joints_[index];
    uint32_t arb_id;
    uint8_t data[8];
    RobstrideProtocol::buildWriteParam(arb_id, data, static_cast<uint8_t>(j.motor_id),
                                       param_id, value);

    // Write param gets feedback response
    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;
    return rsSendAndRecv(arb_id, data, &rx_id, rx_data, &rx_len);
}

// ════════════════════════════════════════════════════════════════
//  Utility
// ════════════════════════════════════════════════════════════════

void IntegratedDriver::clearReceiveBuffer()
{
    uint32_t rx_id;
    uint8_t data[8], len;
    int cleared = 0;
    while (can_.receiveFrame(can_channel_, rx_id, data, len, 1) && cleared < 200) {
        cleared++;
    }
}

}  // namespace arm_sdk
