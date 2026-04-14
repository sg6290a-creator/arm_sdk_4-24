/**
 * ============================================================================
 * ArmDriver — Implementation
 * ============================================================================
 */

#include "arm_sdk/arm_driver.hpp"

#include <chrono>
#include <cstdio>

namespace arm_sdk
{

ArmDriver::ArmDriver() = default;

ArmDriver::~ArmDriver()
{
    cleanup();
}

void ArmDriver::setLogCallback(LogCallback cb)
{
    log_cb_ = std::move(cb);
}

void ArmDriver::log(int level, const std::string& msg)
{
    if (log_cb_) {
        log_cb_(level, msg);
    } else {
        const char* prefix = (level == LOG_ERROR) ? "[ERROR]" :
                             (level == LOG_WARN)  ? "[WARN]"  : "[INFO]";
        fprintf(stderr, "[ArmDriver] %s %s\n", prefix, msg.c_str());
    }
}

// ════════════════════════════════════════════════════════════════
//  Lifecycle
// ════════════════════════════════════════════════════════════════

bool ArmDriver::configure(const ArmDriverConfig& config)
{
    config_ = config;
    can_channel_ = config.can_channel;

    // Step 1: Load USBCAN library
    log(LOG_INFO, "[1/5] Loading " + config.lib_path + "...");
    if (!can_.loadLibrary(config.lib_path)) {
        log(LOG_ERROR, "Failed to load USBCAN library");
        return false;
    }

    // Step 2: Open device
    log(LOG_INFO, "[2/5] Opening USBCAN device (type=" +
        std::to_string(config.device_type) + ", index=" +
        std::to_string(config.device_index) + ")...");
    if (!can_.openDevice(config.device_type, config.device_index)) {
        log(LOG_ERROR, "Failed to open USBCAN device");
        can_.unloadLibrary();
        return false;
    }

    // Step 3: Init and start CAN channel
    uint16_t timing = USBCANDevice::baudrateToTiming(config.baudrate);
    log(LOG_INFO, "[3/5] Initializing CAN channel " +
        std::to_string(config.can_channel) +
        " (timing=0x" + ([&]{
            char buf[8]; snprintf(buf, sizeof(buf), "%04X", timing); return std::string(buf);
        })() + ")...");

    if (!can_.initChannel(config.can_channel, timing)) {
        log(LOG_ERROR, "Failed to init CAN channel");
        can_.closeDevice();
        can_.unloadLibrary();
        return false;
    }
    if (!can_.startChannel(config.can_channel)) {
        log(LOG_ERROR, "Failed to start CAN channel");
        can_.closeDevice();
        can_.unloadLibrary();
        return false;
    }

    // Step 4: Initialize joint data
    joints_.resize(config.joints.size());
    for (size_t i = 0; i < config.joints.size(); ++i) {
        joints_[i].joint_name       = config.joints[i].name;
        joints_[i].actuator_id      = config.joints[i].actuator_id;
        joints_[i].torque_constant  = config.joints[i].torque_constant;
    }

    // Step 5: Test motors and set acceleration
    log(LOG_INFO, "[4/5] Testing motor communication...");
    for (size_t i = 0; i < joints_.size(); ++i) {
        if (readMotor(i)) {
            char buf[128];
            snprintf(buf, sizeof(buf), "  Motor '%s' (ID=%d): %.3f rad (%.1f deg)",
                     joints_[i].joint_name.c_str(), joints_[i].actuator_id,
                     joints_[i].position_state,
                     RMDProtocol::radToDeg(joints_[i].position_state));
            log(LOG_INFO, buf);
        } else {
            log(LOG_ERROR, "Motor '" + joints_[i].joint_name + "' (ID=" +
                std::to_string(joints_[i].actuator_id) + ") did not respond");
            can_.closeDevice();
            can_.unloadLibrary();
            return false;
        }
    }

    log(LOG_INFO, "[5/5] Setting motor acceleration (" +
        std::to_string(config.motor_acceleration) + " dps²)...");
    for (auto& j : joints_) {
        if (setAcceleration(j.actuator_id, config.motor_acceleration)) {
            log(LOG_INFO, "  Motor " + std::to_string(j.actuator_id) +
                ": accel set to " + std::to_string(config.motor_acceleration) + " dps²");
        } else {
            log(LOG_WARN, "  Motor " + std::to_string(j.actuator_id) +
                ": failed to set acceleration (non-critical)");
        }
    }

    log(LOG_INFO, "Configuration complete: " + std::to_string(joints_.size()) + " motors on CAN ch" +
        std::to_string(can_channel_));
    return true;
}

bool ArmDriver::activate()
{
    // Read current positions
    log(LOG_INFO, "Reading current motor positions...");
    for (size_t i = 0; i < joints_.size(); ++i) {
        if (readMotor(i)) {
            char buf[128];
            snprintf(buf, sizeof(buf), "  '%s': %.3f rad (%.1f deg)",
                     joints_[i].joint_name.c_str(), joints_[i].position_state,
                     RMDProtocol::radToDeg(joints_[i].position_state));
            log(LOG_INFO, buf);
        }
    }

    // Initialize commands to current positions
    for (auto& j : joints_) {
        j.position_command = j.position_state;
        j.velocity_command = 0.0;
        j.effort_command   = 0.0;
    }

    log(LOG_INFO, "Activation complete");
    return true;
}

void ArmDriver::deactivate()
{
    stopHoldingThread();
    log(LOG_INFO, "Deactivated");
}

void ArmDriver::cleanup()
{
    stopHoldingThread();
    can_.closeDevice();
    can_.unloadLibrary();
}

// ════════════════════════════════════════════════════════════════
//  Communication
// ════════════════════════════════════════════════════════════════

bool ArmDriver::readMotor(size_t index)
{
    if (index >= joints_.size()) return false;

    auto& j = joints_[index];
    uint8_t cmd[8];
    RMDProtocol::buildReadStatus2(cmd);
    uint32_t tx_id = RMDProtocol::txId(j.actuator_id);
    uint32_t expected_rx = RMDProtocol::rxId(j.actuator_id);

    if (!can_.sendFrame(can_channel_, tx_id, cmd, 8)) {
        return false;
    }

    // Poll for response
    uint32_t rx_id;
    uint8_t rx_data[8];
    uint8_t rx_len;

    for (int attempt = 0; attempt < 20; ++attempt) {
        if (can_.receiveFrame(can_channel_, rx_id, rx_data, rx_len, 1)) {
            if (rx_id == expected_rx) {
                MotorState state = RMDProtocol::parseStatus2(rx_data, rx_len, j.torque_constant);
                if (state.valid) {
                    j.position_state = state.position_rad;
                    j.velocity_state = state.velocity_rads;
                    j.effort_state   = state.effort_nm;
                    j.temperature    = state.temperature;
                    return true;
                }
            }
        }
    }

    return false;
}

bool ArmDriver::writeMotor(size_t index)
{
    if (index >= joints_.size()) return false;

    auto& j = joints_[index];
    uint8_t cmd[8];
    MotorCommand mc;
    mc.position_rad    = j.position_command;
    mc.velocity_rads   = j.velocity_command;
    mc.default_vel_dps = config_.default_velocity_dps;
    mc.max_vel_dps     = config_.max_velocity_dps;

    RMDProtocol::buildPositionCtrl2(cmd, mc);
    return can_.sendFrame(can_channel_, RMDProtocol::txId(j.actuator_id), cmd, 8);
}

bool ArmDriver::readAll()
{
    // Optimized batch: send all 0x9C requests, then collect responses

    // Step 1: Send all read commands
    for (auto& j : joints_) {
        uint8_t cmd[8];
        RMDProtocol::buildReadStatus2(cmd);
        can_.sendFrame(can_channel_, RMDProtocol::txId(j.actuator_id), cmd, 8);
    }

    // Step 2: Small delay for responses
    std::this_thread::sleep_for(std::chrono::microseconds(500));

    // Step 3: Collect responses
    uint32_t rx_id;
    uint8_t rx_data[8];
    uint8_t rx_len;

    size_t responses_needed = joints_.size();
    int attempts = 0;
    const int max_attempts = static_cast<int>(responses_needed * 15);

    while (responses_needed > 0 && attempts < max_attempts) {
        if (can_.receiveFrame(can_channel_, rx_id, rx_data, rx_len, 1)) {
            for (auto& j : joints_) {
                if (rx_id == RMDProtocol::rxId(j.actuator_id)) {
                    MotorState state = RMDProtocol::parseStatus2(rx_data, rx_len, j.torque_constant);
                    if (state.valid) {
                        j.position_state = state.position_rad;
                        j.velocity_state = state.velocity_rads;
                        j.effort_state   = state.effort_nm;
                        j.temperature    = state.temperature;
                        responses_needed--;
                    }
                    break;
                }
            }
        }
        attempts++;
    }

    return responses_needed < joints_.size();  // at least one succeeded
}

void ArmDriver::writeAll()
{
    for (size_t i = 0; i < joints_.size(); ++i) {
        writeMotor(i);
    }
}

bool ArmDriver::setAcceleration(int actuator_id, uint32_t accel_dps2)
{
    uint8_t cmd[8];
    RMDProtocol::buildSetAcceleration(cmd, accel_dps2);

    if (!can_.sendFrame(can_channel_, RMDProtocol::txId(actuator_id), cmd, 8)) {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    clearReceiveBuffer();
    return true;
}

void ArmDriver::clearReceiveBuffer()
{
    uint32_t rx_id;
    uint8_t data[8];
    uint8_t len;
    int cleared = 0;
    while (can_.receiveFrame(can_channel_, rx_id, data, len, 1) && cleared < 100) {
        cleared++;
    }
}

// ════════════════════════════════════════════════════════════════
//  Holding Thread
// ════════════════════════════════════════════════════════════════

void ArmDriver::startHoldingThread()
{
    stop_holding_.store(false);
    holding_thread_ = std::thread(&ArmDriver::holdingThreadFunc, this);
}

void ArmDriver::stopHoldingThread()
{
    stop_holding_.store(true);
    if (holding_thread_.joinable()) {
        holding_thread_.join();
    }
}

void ArmDriver::holdingThreadFunc()
{
    while (!stop_holding_.load()) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (auto& j : joints_) {
                j.position_command = 0.0;
                j.velocity_command = RMDProtocol::dpsToRadS(10.0);
                writeMotor(static_cast<size_t>(&j - &joints_[0]));
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

}  // namespace arm_sdk
