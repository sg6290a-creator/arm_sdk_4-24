/**
 * ============================================================================
 * IntegratedDriver — Implementation (SocketCAN / CANable v2.0)
 * ============================================================================
 *
 * RMD + Robstride 모터를 1개의 SocketCAN 인터페이스에서 통합 제어.
 * CAN 프레임 구분: extended flag (false=RMD, true=Robstride)
 *
 * ============================================================================
 */

#include "arm_sdk/integrated_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <thread>

namespace arm_sdk
{

namespace
{

constexpr int kAsyncReaderIdleSleepUs = 250;

MotorState parseRmdFeedbackFrame(const uint8_t rx_data[8], uint8_t rx_len,
                                 double torque_constant)
{
    if (rx_len < 8) {
        return {};
    }

    if (rx_data[0] == RMD_CMD_READ_STATUS2) {
        return RMDProtocol::parseStatus2(rx_data, rx_len, torque_constant);
    }

    if (rx_data[0] != RMD_CMD_POSITION_CTRL2) {
        return {};
    }

    MotorState state;
    state.temperature = static_cast<int8_t>(rx_data[1]);

    const int16_t current_raw = static_cast<int16_t>(
        rx_data[2] | (uint16_t(rx_data[3]) << 8));
    const int16_t speed_raw = static_cast<int16_t>(
        rx_data[4] | (uint16_t(rx_data[5]) << 8));
    const uint16_t encoder_raw = static_cast<uint16_t>(
        rx_data[6] | (uint16_t(rx_data[7]) << 8));

    const double position_deg = static_cast<double>(encoder_raw) * 360.0 / 65536.0;
    state.position_rad = RMDProtocol::degToRad(position_deg);
    state.velocity_rads = RMDProtocol::dpsToRadS(static_cast<double>(speed_raw));
    state.effort_nm = current_raw * 0.01 * torque_constant;
    state.valid = true;
    return state;
}

}  // namespace

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

void IntegratedDriver::startAsyncFeedback()
{
    if (async_reader_.isRunning()) {
        return;
    }

    async_reader_.setFrameHandler(
        [this](uint32_t rx_id, const uint8_t* rx_data, uint8_t rx_len, bool ext) {
            handleAsyncFrame(rx_id, rx_data, rx_len, ext);
        });

    if (!async_reader_.start(&can_, &io_mutex_, kAsyncReaderIdleSleepUs)) {
        log(LOG_WARN, "Failed to start async CAN reader; falling back to stale cached reads");
    }
}

void IntegratedDriver::stopAsyncFeedback()
{
    async_reader_.stop();
}

void IntegratedDriver::markJointFeedback(UnifiedJoint& joint)
{
    joint.online = true;
    joint.feedback_valid = true;
    joint.last_feedback_time = std::chrono::steady_clock::now();
}

bool IntegratedDriver::isJointFeedbackFresh(
    const UnifiedJoint& joint,
    const std::chrono::steady_clock::time_point& now) const
{
    if (!joint.feedback_valid) {
        return false;
    }

    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - joint.last_feedback_time).count();
    const int max_age_ms = std::max(20, config_.read_deadline_ms * 4);
    return age_ms <= max_age_ms;
}

bool IntegratedDriver::shouldPollJoint(
    const UnifiedJoint& joint,
    const std::chrono::steady_clock::time_point& now) const
{
    if (!joint.feedback_valid) {
        return true;
    }

    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - joint.last_feedback_time).count();
    const int poll_age_ms = std::max(2, config_.read_deadline_ms);
    return age_ms >= poll_age_ms;
}

void IntegratedDriver::handleAsyncFrame(
    uint32_t rx_id, const uint8_t* rx_data, uint8_t rx_len, bool ext)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!ext) {
        for (auto& j : joints_) {
            if (j.motor_type != MotorType::RMD) continue;
            if (rx_id != RMDProtocol::rxId(j.motor_id)) continue;

            MotorState s = parseRmdFeedbackFrame(rx_data, rx_len, j.torque_constant);
            if (!s.valid) break;

            j.position_rad = s.position_rad;
            j.velocity_rads = s.velocity_rads;
            j.effort_nm = s.effort_nm;
            j.temperature = s.temperature;
            markJointFeedback(j);
            break;
        }
        return;
    }

    if (RobstrideProtocol::getMsgType(rx_id) != RS_MSG_FEEDBACK) {
        return;
    }

    const uint8_t fb_motor_id = static_cast<uint8_t>(rx_id & 0xFF);
    for (auto& j : joints_) {
        if (j.motor_type != MotorType::ROBSTRIDE) continue;
        if (static_cast<uint8_t>(j.motor_id) != fb_motor_id) continue;

        RobstrideState s = RobstrideProtocol::parseFeedback(
            rx_id, rx_data,
            j.rs_torque_min, j.rs_torque_max,
            j.rs_vel_min,    j.rs_vel_max);
        if (!s.valid) break;

        j.position_rad = static_cast<double>(s.position_rad);
        j.velocity_rads = static_cast<double>(s.velocity_rads);
        j.effort_nm = static_cast<double>(s.torque_nm);
        j.temperature = static_cast<double>(s.temperature);
        j.rs_mode = s.mode;
        j.rs_error_bits = s.error_bits;
        markJointFeedback(j);
        break;
    }
}

// ════════════════════════════════════════════════════════════════
//  Lifecycle
// ════════════════════════════════════════════════════════════════

bool IntegratedDriver::configure(const IntegratedDriverConfig& config)
{
    stopAsyncFeedback();
    config_ = config;

    // ── Step 1: Open SocketCAN ─────────────────────────────
    log(LOG_INFO, "[1/3] Opening SocketCAN interface: " + config.can_if + "...");
    if (!can_.open(config.can_if)) {
        log(LOG_ERROR, "Failed to open SocketCAN interface: " + config.can_if);
        return false;
    }
    log(LOG_INFO, "  [OK] " + config.can_if + " open");

    // ── Step 2: Initialize joints ──────────────────────────
    joints_.resize(config.joints.size());
    for (size_t i = 0; i < config.joints.size(); ++i) {
        const auto& def = config.joints[i];
        auto& j = joints_[i];
        j.name                = def.name;
        j.motor_type          = def.motor_type;
        j.motor_id            = def.motor_id;
        j.required            = def.required;
        j.online              = false;
        j.torque_constant     = def.torque_constant;
        j.robstride_max_speed = def.max_speed_rads;
        j.feedback_valid      = false;
    }

    clearReceiveBuffer();

    int rmd_count = 0, rs_count = 0;
    for (const auto& j : joints_) {
        if (j.motor_type == MotorType::RMD) rmd_count++;
        else rs_count++;
    }

    // ── Step 3: Test communication ─────────────────────────
    log(LOG_INFO, "[2/3] Testing " + std::to_string(joints_.size()) +
        " motors (" + std::to_string(rmd_count) + " RMD + " +
        std::to_string(rs_count) + " Robstride)...");

    for (size_t i = 0; i < joints_.size(); ++i) {
        auto& j = joints_[i];
        bool ok = false;

        if (j.motor_type == MotorType::RMD) {
            ok = readRmdMotor(i);
        } else {
            ok = enableRobstride(i);
            if (ok) {
                j.enabled = true;
                disableRobstride(i);
                j.enabled = false;
            }
        }

        if (ok) {
            j.online = true;
            char buf[128];
            snprintf(buf, sizeof(buf),
                     "  [OK] '%s' (%s, ID=%d): pos=%.2f°",
                     j.name.c_str(), motorTypeString(j.motor_type),
                     j.motor_id, j.position_rad * 180.0 / M_PI);
            log(LOG_INFO, buf);
        } else if (j.required) {
            log(LOG_ERROR, "  [FAIL] '" + j.name + "' (" +
                motorTypeString(j.motor_type) + ", ID=" +
                std::to_string(j.motor_id) + ") — no response");
            can_.close();
            return false;
        } else {
            log(LOG_WARN, "  [SKIP] optional '" + j.name + "' (" +
                motorTypeString(j.motor_type) + ", ID=" +
                std::to_string(j.motor_id) + ") — no response");
            j.online = false;
            j.enabled = false;
            j.feedback_valid = false;
        }
    }

    log(LOG_INFO, "[3/3] Configuration complete: " +
        std::to_string(joints_.size()) + " motors on " + config.can_if);
    return true;
}

bool IntegratedDriver::activate()
{
    log(LOG_INFO, "Activating motors...");

    for (size_t i = 0; i < joints_.size(); ++i) {
        auto& j = joints_[i];
        if (!j.online) {
            if (!j.required) {
                log(LOG_WARN, "  optional joint '" + j.name + "' remains offline; skipping activate");
                continue;
            }
            log(LOG_WARN, "  required joint '" + j.name + "' is offline during activate");
            continue;
        }

        if (j.motor_type == MotorType::RMD)
        {
            if (readRmdMotor(i)) {
                j.position_command = j.position_rad;
                j.enabled = true;
                setRmdAcceleration(i, config_.rmd_acceleration);
                char buf[128];
                snprintf(buf, sizeof(buf), "  RMD '%s' (ID=%d): %.2f° — ready",
                         j.name.c_str(), j.motor_id, j.position_rad * 180.0 / M_PI);
                log(LOG_INFO, buf);
            } else {
                log(LOG_WARN, "  RMD '" + j.name + "' read failed during activate");
            }
        }
        else  // ROBSTRIDE
        {
            disableRobstride(i);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            writeRobstrideParam(i, RS_PARAM_RUN_MODE, RS_MODE_POSITION);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            writeRobstrideParam(i, RS_PARAM_LIMIT_SPD, j.robstride_max_speed);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            if (enableRobstride(i)) {
                j.enabled = true;

                float mech_pos = 0;
                if (readRobstrideParam(i, RS_PARAM_MECH_POS, mech_pos)) {
                    j.position_rad = static_cast<double>(mech_pos);
                    j.position_command = j.position_rad;
                }

                char buf[128];
                snprintf(buf, sizeof(buf),
                         "  Robstride '%s' (ID=%d): %.2f° — position mode, ready",
                         j.name.c_str(), j.motor_id, j.position_rad * 180.0 / M_PI);
                log(LOG_INFO, buf);
            } else {
                log(LOG_WARN, "  Robstride '" + j.name + "' enable failed");
            }
        }
    }

    log(LOG_INFO, "Activation complete");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    clearReceiveBuffer();
    startAsyncFeedback();
    return true;
}

void IntegratedDriver::deactivate()
{
    stopAsyncFeedback();
    log(LOG_INFO, "Deactivating motors...");

    for (size_t i = 0; i < joints_.size(); ++i) {
        auto& j = joints_[i];
        if (!j.online) {
            j.enabled = false;
            continue;
        }
        if (j.motor_type == MotorType::ROBSTRIDE && j.enabled) {
            disableRobstride(i);
            j.enabled = false;
            log(LOG_INFO, "  Robstride '" + j.name + "' disabled");
        }
        j.enabled = false;
    }

    log(LOG_INFO, "Deactivation complete");
}

void IntegratedDriver::cleanup()
{
    stopAsyncFeedback();
    deactivate();
    can_.close();
}

// ════════════════════════════════════════════════════════════════
//  Communication — Read
// ════════════════════════════════════════════════════════════════

bool IntegratedDriver::readAll()
{
    struct PollRequest {
        uint32_t id = 0;
        uint8_t data[8] = {0};
        uint8_t len = 8;
        bool extended = false;
    };

    std::vector<PollRequest> polls;
    const auto now = std::chrono::steady_clock::now();
    bool all_fresh = true;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        polls.reserve(joints_.size());

        for (const auto& j : joints_) {
            if (!j.online && !j.required) {
                continue;
            }

            if (!isJointFeedbackFresh(j, now)) {
                all_fresh = false;
            }

            if (j.online && shouldPollJoint(j, now)) {
                PollRequest req;
                if (j.motor_type == MotorType::RMD) {
                    RMDProtocol::buildReadStatus2(req.data);
                    req.id = RMDProtocol::txId(j.motor_id);
                    req.extended = false;
                } else {
                    RobstrideProtocol::buildEnable(
                        req.id, req.data, static_cast<uint8_t>(j.motor_id));
                    req.extended = true;
                }
                polls.push_back(req);
            }
        }
    }

    if (all_fresh) {
        return true;
    }

    // ── TX only: stale joints are polled, RX is handled by async thread ─────
    for (const auto& poll : polls) {
        std::lock_guard<std::mutex> io_lock(io_mutex_);
        if (!can_.sendFrame(poll.id, poll.data, poll.len, poll.extended)) {
            return false;
        }
    }

    return false;
}

bool IntegratedDriver::readMotor(size_t index)
{
    if (index >= joints_.size()) return false;
    if (!joints_[index].online) {
        return !joints_[index].required;
    }

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
    if (!joints_[index].online) {
        return !joints_[index].required;
    }

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

    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;

    std::lock_guard<std::mutex> io_lock(io_mutex_);
    if (!can_.sendFrame(tx_id, cmd, 8, false))  // standard CAN
        return false;

    for (int attempt = 0; attempt < 10; ++attempt) {
        bool ext = false;
        if (can_.receiveFrame(rx_id, rx_data, rx_len, 1, &ext)) {
            if (!ext && rx_id == expected_rx) {
                MotorState state = RMDProtocol::parseStatus2(rx_data, rx_len, j.torque_constant);
                if (state.valid) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    j.position_rad  = state.position_rad;
                    j.velocity_rads = state.velocity_rads;
                    j.effort_nm     = state.effort_nm;
                    j.temperature   = state.temperature;
                    markJointFeedback(j);
                    return true;
                }
            }
        }
    }
    return false;
}

bool IntegratedDriver::writeRmdMotor(size_t index)
{
    int motor_id = 0;
    uint8_t cmd[8];
    MotorCommand mc;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto& j = joints_[index];
        motor_id = j.motor_id;
        mc.position_rad = j.position_command;
        mc.velocity_rads = j.velocity_command;
        mc.default_vel_dps = config_.rmd_default_vel_dps;
        mc.max_vel_dps = config_.rmd_max_vel_dps;
    }

    RMDProtocol::buildPositionCtrl2(cmd, mc);
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    return can_.sendFrame(RMDProtocol::txId(motor_id), cmd, 8, false);
}

bool IntegratedDriver::setRmdAcceleration(size_t index, uint32_t accel_dps2)
{
    if (index >= joints_.size() || joints_[index].motor_type != MotorType::RMD)
        return false;
    if (!joints_[index].online) {
        return !joints_[index].required;
    }

    auto& j = joints_[index];
    uint8_t cmd[8];
    RMDProtocol::buildSetAcceleration(cmd, accel_dps2);

    std::lock_guard<std::mutex> io_lock(io_mutex_);
    if (!can_.sendFrame(RMDProtocol::txId(j.motor_id), cmd, 8, false))
        return false;

    // Consume ACK to keep receive buffer clean for subsequent reads
    uint32_t rx_id; uint8_t rx_data[8], rx_len; bool ext = false;
    for (int attempt = 0; attempt < 10; ++attempt) {
        if (can_.receiveFrame(rx_id, rx_data, rx_len, 10, &ext)) {
            if (!ext && rx_id == RMDProtocol::rxId(j.motor_id)) break;
        } else {
            break;
        }
    }
    return true;
}

// ════════════════════════════════════════════════════════════════
//  Internal — Robstride
// ════════════════════════════════════════════════════════════════

bool IntegratedDriver::rsSendAndRecv(uint32_t tx_id, const uint8_t* tx_data,
                                     uint32_t* rx_id, uint8_t* rx_data,
                                     uint8_t* rx_len, int timeout_ms)
{
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    if (!can_.sendFrame(tx_id, tx_data, 8, true))  // extended CAN
        return false;

    uint32_t id;
    uint8_t data[8], len;

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        bool ext = false;
        if (can_.receiveFrame(id, data, len, 2, &ext)) {
            if (ext) {
                uint8_t resp_motor = static_cast<uint8_t>((id >> 8) & 0xFF);
                uint8_t sent_motor = static_cast<uint8_t>(tx_id & 0xFF);
                if (resp_motor != sent_motor) continue;

                if (rx_id)   *rx_id = id;
                if (rx_data) std::memcpy(rx_data, data, 8);
                if (rx_len)  *rx_len = len;
                return true;
            }
        }
    }
    return false;
}

bool IntegratedDriver::readRobstrideMotor(size_t index)
{
    auto& j = joints_[index];

    uint32_t arb_id;
    uint8_t data[8];
    RobstrideProtocol::buildEnable(arb_id, data, static_cast<uint8_t>(j.motor_id));

    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;

    if (!rsSendAndRecv(arb_id, data, &rx_id, rx_data, &rx_len))
        return false;

    RobstrideState state = RobstrideProtocol::parseFeedback(
        rx_id, rx_data,
        j.rs_torque_min, j.rs_torque_max,
        j.rs_vel_min,    j.rs_vel_max);
    if (state.valid) {
        std::lock_guard<std::mutex> lock(mutex_);
        j.position_rad  = static_cast<double>(state.position_rad);
        j.velocity_rads = static_cast<double>(state.velocity_rads);
        j.effort_nm     = static_cast<double>(state.torque_nm);
        j.temperature   = static_cast<double>(state.temperature);
        j.rs_mode       = state.mode;
        j.rs_error_bits = state.error_bits;
        markJointFeedback(j);
        return true;
    }
    return false;
}

bool IntegratedDriver::writeRobstrideMotor(size_t index)
{
    int motor_id = 0;
    float target = 0.0f;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto& j = joints_[index];
        motor_id = j.motor_id;
        target = static_cast<float>(j.position_command);
    }

    uint32_t arb_id;
    uint8_t data[8];
    RobstrideProtocol::buildWriteParam(
        arb_id, data, static_cast<uint8_t>(motor_id), RS_PARAM_LOC_REF, target);

    std::lock_guard<std::mutex> io_lock(io_mutex_);
    return can_.sendFrame(arb_id, data, 8, true);
}

bool IntegratedDriver::enableRobstride(size_t index)
{
    if (index >= joints_.size() || joints_[index].motor_type != MotorType::ROBSTRIDE)
        return false;
    if (!joints_[index].online && !joints_[index].required) {
        return true;
    }

    auto& j = joints_[index];
    uint32_t arb_id;
    uint8_t data[8];
    RobstrideProtocol::buildEnable(arb_id, data, static_cast<uint8_t>(j.motor_id));

    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;

    if (!rsSendAndRecv(arb_id, data, &rx_id, rx_data, &rx_len))
        return false;

    RobstrideState state = RobstrideProtocol::parseFeedback(
        rx_id, rx_data,
        j.rs_torque_min, j.rs_torque_max,
        j.rs_vel_min,    j.rs_vel_max);
    if (state.valid) {
        std::lock_guard<std::mutex> lock(mutex_);
        j.position_rad  = static_cast<double>(state.position_rad);
        j.velocity_rads = static_cast<double>(state.velocity_rads);
        j.effort_nm     = static_cast<double>(state.torque_nm);
        j.temperature   = static_cast<double>(state.temperature);
        j.rs_mode       = state.mode;
        j.rs_error_bits = state.error_bits;
        j.enabled       = true;
        markJointFeedback(j);
    }
    return state.valid;
}

bool IntegratedDriver::disableRobstride(size_t index)
{
    if (index >= joints_.size() || joints_[index].motor_type != MotorType::ROBSTRIDE)
        return false;
    if (!joints_[index].online) {
        return !joints_[index].required;
    }

    int motor_id = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        motor_id = joints_[index].motor_id;
    }
    uint32_t arb_id;
    uint8_t data[8];
    RobstrideProtocol::buildDisable(arb_id, data, static_cast<uint8_t>(motor_id));

    std::lock_guard<std::mutex> io_lock(io_mutex_);
    bool ok = can_.sendFrame(arb_id, data, 8, true);
    if (ok) {
        std::lock_guard<std::mutex> lock(mutex_);
        joints_[index].enabled = false;
    }
    return ok;
}

bool IntegratedDriver::setRobstrideMode(size_t index, uint8_t mode)
{
    if (index >= joints_.size() || joints_[index].motor_type != MotorType::ROBSTRIDE)
        return false;
    if (!joints_[index].online) {
        return !joints_[index].required;
    }

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
    if (!joints_[index].online) {
        return !joints_[index].required;
    }

    auto& j = joints_[index];
    uint32_t arb_id;
    uint8_t data[8];
    RobstrideProtocol::buildReadParam(arb_id, data, static_cast<uint8_t>(j.motor_id), param_id);

    std::lock_guard<std::mutex> io_lock(io_mutex_);
    if (!can_.sendFrame(arb_id, data, 8, true))
        return false;

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);

    while (std::chrono::steady_clock::now() < deadline) {
        uint32_t rx_id;
        uint8_t rx_data[8], rx_len;
        bool ext = false;

        if (can_.receiveFrame(rx_id, rx_data, rx_len, 5, &ext)) {
            if (!ext) continue;

            uint8_t msg_type = RobstrideProtocol::getMsgType(rx_id);
            if (msg_type == RS_MSG_FEEDBACK) continue;

            if (RobstrideProtocol::parseParamResponse(rx_data, param_id, value))
                return true;
        }
    }
    return false;
}

bool IntegratedDriver::writeRobstrideParam(size_t index, uint16_t param_id, float value)
{
    if (index >= joints_.size() || joints_[index].motor_type != MotorType::ROBSTRIDE)
        return false;
    if (!joints_[index].online) {
        return !joints_[index].required;
    }

    auto& j = joints_[index];
    uint32_t arb_id;
    uint8_t data[8];
    RobstrideProtocol::buildWriteParam(arb_id, data, static_cast<uint8_t>(j.motor_id),
                                       param_id, value);

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
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    while (can_.receiveFrame(rx_id, data, len, 1) && cleared < 200) {
        cleared++;
    }
}

}  // namespace arm_sdk
