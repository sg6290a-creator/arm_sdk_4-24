/**
 * ============================================================================
 * test_robstride — USBCAN으로 Robstride 모터 1개 테스트 (ROS 없이)
 * ============================================================================
 *
 * 기능:
 *   1) Enable 모터
 *   2) 상태 읽기 (각도, 속도, 토크, 온도)
 *   3) 위치 모드로 목표 각도 이동
 *   4) Disable 모터
 *
 * 사용법:
 *   sudo ./test_robstride [motor_id] [can_channel]
 *   sudo ./test_robstride           ← 기본: motor_id=1, channel=0
 *   sudo ./test_robstride 1 0
 *
 * 빌드:
 *   cd C++/build && cmake .. && make test_robstride
 *
 * ============================================================================
 */

#include "arm_sdk/usbcan_device.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <cmath>
#include <chrono>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ════════════════════════════════════════════════════════════════════
// Robstride Protocol Constants
// ════════════════════════════════════════════════════════════════════

// Communication types (msg_type in bits 28-24 of extended CAN ID)
constexpr uint8_t MSG_GET_ID      = 0x00;
constexpr uint8_t MSG_CONTROL     = 0x01;  // Motion control (MIT mode)
constexpr uint8_t MSG_FEEDBACK    = 0x02;  // Motor feedback
constexpr uint8_t MSG_ENABLE      = 0x03;
constexpr uint8_t MSG_DISABLE     = 0x04;
constexpr uint8_t MSG_SET_ZERO    = 0x06;
constexpr uint8_t MSG_SET_ID      = 0x07;
constexpr uint8_t MSG_READ_PARAM  = 0x11;
constexpr uint8_t MSG_WRITE_PARAM = 0x12;

// Parameter indices
constexpr uint16_t PARAM_RUN_MODE    = 0x7005;
constexpr uint16_t PARAM_SPD_REF     = 0x700A;
constexpr uint16_t PARAM_LIMIT_TORQUE= 0x700B;
constexpr uint16_t PARAM_LOC_REF     = 0x7016;
constexpr uint16_t PARAM_LIMIT_SPD   = 0x7017;
constexpr uint16_t PARAM_LIMIT_CUR   = 0x7018;
constexpr uint16_t PARAM_MECH_POS    = 0x7019;
constexpr uint16_t PARAM_MECH_VEL    = 0x701B;
constexpr uint16_t PARAM_VBUS        = 0x701C;
constexpr uint16_t PARAM_LOC_KP      = 0x701E;
constexpr uint16_t PARAM_SPD_KP      = 0x701F;
constexpr uint16_t PARAM_SPD_KI      = 0x7020;

// Run modes
constexpr uint8_t MODE_MOTION   = 0;  // MIT mode (운동 제어)
constexpr uint8_t MODE_POSITION = 1;  // 위치 모드
constexpr uint8_t MODE_SPEED    = 2;  // 속도 모드
constexpr uint8_t MODE_CURRENT  = 3;  // 전류 모드

// Feedback ranges (motor_model=1, RS01 series)
constexpr float P_MIN = -12.5f;
constexpr float P_MAX =  12.5f;
constexpr float V_MIN = -44.0f;
constexpr float V_MAX =  44.0f;
constexpr float T_MIN = -17.0f;
constexpr float T_MAX =  17.0f;

constexpr uint8_t HOST_CAN_ID = 0x00;  // Master CAN ID

// ════════════════════════════════════════════════════════════════════
// Helper functions
// ════════════════════════════════════════════════════════════════════

/// Build extended CAN arbitration ID: [msg_type:5][data1:16][motor_id:8]
static uint32_t makeArbId(uint8_t msg_type, uint16_t data1, uint8_t motor_id)
{
    return (static_cast<uint32_t>(msg_type) << 24) |
           (static_cast<uint32_t>(data1) << 8) |
           motor_id;
}

/// uint16 to float mapping
static float uint16ToFloat(uint16_t x, float x_min, float x_max)
{
    return (x_max - x_min) * static_cast<float>(x) / 65535.0f + x_min;
}

/// float to uint16 mapping
static uint16_t floatToUint16(float x, float x_min, float x_max)
{
    if (x > x_max) x = x_max;
    if (x < x_min) x = x_min;
    return static_cast<uint16_t>((x - x_min) / (x_max - x_min) * 65535.0f);
}

/// Parse feedback response from CAN frame
struct RobstrideFeedback {
    uint8_t motor_id;
    uint8_t mode;        // 0=Reset, 1=Calibration, 2=Run
    uint8_t error_bits;
    float   angle;       // rad
    float   velocity;    // rad/s
    float   torque;      // Nm
    float   temp;        // °C
};

static RobstrideFeedback parseFeedback(uint32_t arb_id, const uint8_t* data)
{
    RobstrideFeedback fb;
    fb.motor_id   = (arb_id >> 8) & 0xFF;
    fb.error_bits = (arb_id >> 16) & 0x3F;
    fb.mode       = (arb_id >> 22) & 0x03;

    uint16_t angle_raw = (data[0] << 8) | data[1];
    uint16_t vel_raw   = (data[2] << 8) | data[3];
    uint16_t torq_raw  = (data[4] << 8) | data[5];
    uint16_t temp_raw  = (data[6] << 8) | data[7];

    fb.angle    = uint16ToFloat(angle_raw, -4.0f * M_PI, 4.0f * M_PI);
    fb.velocity = uint16ToFloat(vel_raw, -44.0f, 44.0f);     // model 1
    fb.torque   = uint16ToFloat(torq_raw, -17.0f, 17.0f);    // model 1
    fb.temp     = static_cast<float>(temp_raw) / 10.0f;

    return fb;
}

static const char* modeStr(uint8_t mode)
{
    switch (mode) {
        case 0: return "Reset";
        case 1: return "Calibration";
        case 2: return "Run";
        default: return "Unknown";
    }
}

// ════════════════════════════════════════════════════════════════════
// CAN send/recv wrappers
// ════════════════════════════════════════════════════════════════════

static arm_sdk::USBCANDevice g_can;
static uint32_t g_channel = 0;

static bool canSend(uint32_t arb_id, const uint8_t* data, uint8_t len = 8)
{
    return g_can.sendFrame(g_channel, arb_id, data, len, 1);  // extern_flag=1 (extended)
}

static bool canRecv(uint32_t& arb_id, uint8_t* data, uint8_t& len, int timeout_ms = 200)
{
    uint8_t ext;
    return g_can.receiveFrame(g_channel, arb_id, data, len, timeout_ms, &ext);
}

/// Send command and receive feedback
static bool sendAndRecv(uint32_t arb_id, const uint8_t* data, RobstrideFeedback* fb = nullptr)
{
    if (!canSend(arb_id, data))
    {
        printf("  [ERROR] CAN send failed\n");
        return false;
    }

    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;

    if (!canRecv(rx_id, rx_data, rx_len, 500))
    {
        printf("  [ERROR] No response (timeout)\n");
        return false;
    }

    if (fb) *fb = parseFeedback(rx_id, rx_data);
    return true;
}

// ════════════════════════════════════════════════════════════════════
// Motor commands
// ════════════════════════════════════════════════════════════════════

static bool enableMotor(uint8_t motor_id, RobstrideFeedback* fb = nullptr)
{
    uint8_t data[8] = {0};
    uint32_t arb_id = makeArbId(MSG_ENABLE, HOST_CAN_ID, motor_id);
    return sendAndRecv(arb_id, data, fb);
}

static bool disableMotor(uint8_t motor_id, uint8_t clear_error = 0)
{
    uint8_t data[8] = {clear_error, 0, 0, 0, 0, 0, 0, 0};
    uint32_t arb_id = makeArbId(MSG_DISABLE, HOST_CAN_ID, motor_id);
    return canSend(arb_id, data);
}

static bool setZeroPos(uint8_t motor_id)
{
    disableMotor(motor_id, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    uint8_t data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    uint32_t arb_id = makeArbId(MSG_SET_ZERO, HOST_CAN_ID, motor_id);
    canSend(arb_id, data);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    RobstrideFeedback fb;
    enableMotor(motor_id, &fb);
    return true;
}

static bool writeParam(uint8_t motor_id, uint16_t param_id, float value)
{
    uint8_t data[8] = {0};
    data[0] = param_id & 0xFF;
    data[1] = (param_id >> 8) & 0xFF;
    data[2] = 0;
    data[3] = 0;

    if (param_id == PARAM_RUN_MODE)
    {
        data[4] = static_cast<uint8_t>(value);
        data[5] = 0;
        data[6] = 0;
        data[7] = 0;
    }
    else
    {
        memcpy(&data[4], &value, 4);
    }

    uint32_t arb_id = makeArbId(MSG_WRITE_PARAM, HOST_CAN_ID, motor_id);
    return sendAndRecv(arb_id, data);
}

static bool readParam(uint8_t motor_id, uint16_t param_id, float& value)
{
    uint8_t data[8] = {0};
    data[0] = param_id & 0xFF;
    data[1] = (param_id >> 8) & 0xFF;

    uint32_t arb_id = makeArbId(MSG_READ_PARAM, HOST_CAN_ID, motor_id);
    if (!canSend(arb_id, data)) return false;

    // Robstride sends feedback frame first, then param response.
    // Keep reading until we find the matching param response.
    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;

    for (int attempt = 0; attempt < 10; ++attempt)
    {
        if (!canRecv(rx_id, rx_data, rx_len, 500)) return false;

        // Check if this is a param response (msg_type = 0x11 READ_PARAM)
        uint8_t msg_type = (rx_id >> 24) & 0x1F;
        if (msg_type == MSG_FEEDBACK)
        {
            // Feedback frame — skip it, keep reading
            continue;
        }

        // Verify param ID in response
        uint16_t resp_param = rx_data[0] | (rx_data[1] << 8);
        if (resp_param != param_id)
        {
            // Different param — skip (stale from previous call)
            continue;
        }

        // Matched!
        if (param_id == PARAM_RUN_MODE)
        {
            value = static_cast<float>(rx_data[4]);
        }
        else
        {
            memcpy(&value, &rx_data[4], 4);
        }
        return true;
    }

    return false;
}

// ════════════════════════════════════════════════════════════════════
// Signal handler
// ════════════════════════════════════════════════════════════════════

static volatile bool g_running = true;
void signalHandler(int) { g_running = false; }

// ════════════════════════════════════════════════════════════════════
// Main
// ════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[])
{
    signal(SIGINT, signalHandler);

    uint8_t motor_id = (argc >= 2) ? static_cast<uint8_t>(atoi(argv[1])) : 1;
    g_channel        = (argc >= 3) ? static_cast<uint32_t>(atoi(argv[2])) : 0;

    printf("╔══════════════════════════════════════════════╗\n");
    printf("║  Robstride Motor Test (USBCAN)               ║\n");
    printf("╠══════════════════════════════════════════════╣\n");
    printf("║  Motor ID:    %3d                             ║\n", motor_id);
    printf("║  CAN Channel: %3d                             ║\n", g_channel);
    printf("╚══════════════════════════════════════════════╝\n\n");

    // ── Step 1: Open USBCAN ────────────────────────────────
    printf("[1] Loading USBCAN...\n");
    if (!g_can.loadLibrary("libusbcan.so"))
    {
        printf("  [FAIL] Cannot load libusbcan.so\n");
        return 1;
    }

    if (!g_can.openDevice(arm_sdk::USBCAN_II, 0))
    {
        printf("  [FAIL] Cannot open USBCAN device\n");
        return 1;
    }

    uint16_t timing = arm_sdk::USBCANDevice::baudrateToTiming(1000000);
    if (!g_can.initChannel(g_channel, timing) || !g_can.startChannel(g_channel))
    {
        printf("  [FAIL] Cannot init/start CAN channel %d\n", g_channel);
        return 1;
    }
    printf("  [OK] USBCAN ready (1Mbps, channel=%d)\n\n", g_channel);

    // Flush pending frames
    {
        uint32_t id; uint8_t d[8], l;
        while (canRecv(id, d, l, 10)) {}
    }

    // ── Step 2: Enable motor ───────────────────────────────
    printf("[2] Enabling motor %d...\n", motor_id);
    RobstrideFeedback fb;
    if (!enableMotor(motor_id, &fb))
    {
        printf("  [FAIL] Enable failed. Motor connected? ID correct?\n");
        disableMotor(motor_id);
        g_can.closeDevice();
        return 1;
    }
    printf("  [OK] Motor enabled!\n");
    printf("  Mode: %s  Angle: %.2f°  Vel: %.2f rad/s  Torque: %.2f Nm  Temp: %.1f°C\n",
           modeStr(fb.mode),
           fb.angle * 180.0 / M_PI,
           fb.velocity,
           fb.torque,
           fb.temp);
    if (fb.error_bits)
        printf("  ⚠ Error bits: 0x%02X\n", fb.error_bits);
    printf("\n");

    // ── Step 3: Read parameters ────────────────────────────
    printf("[3] Reading parameters...\n");
    float val;
    if (readParam(motor_id, PARAM_RUN_MODE, val))
        printf("  run_mode = %d (%s)\n", (int)val,
               (int)val == 0 ? "Motion" : (int)val == 1 ? "Position" : (int)val == 2 ? "Speed" : "Current");
    if (readParam(motor_id, PARAM_MECH_POS, val))
        printf("  mech_pos = %.4f rad (%.2f°)\n", val, val * 180.0 / M_PI);
    if (readParam(motor_id, PARAM_MECH_VEL, val))
        printf("  mech_vel = %.4f rad/s\n", val);
    if (readParam(motor_id, PARAM_VBUS, val))
        printf("  vbus     = %.2f V\n", val);
    printf("\n");

    // ── Step 4: Set position mode and move ─────────────────
    printf("[4] Setting Position Mode...\n");

    // Disable first, change mode, re-enable
    disableMotor(motor_id, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    writeParam(motor_id, PARAM_RUN_MODE, MODE_POSITION);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    writeParam(motor_id, PARAM_LIMIT_SPD, 5.0f);   // 5 rad/s max speed
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    enableMotor(motor_id, &fb);
    printf("  Mode after set: %s\n", modeStr(fb.mode));

    // Read current position
    float start_pos = 0;
    readParam(motor_id, PARAM_MECH_POS, start_pos);
    printf("  Current position: %.2f° (%.4f rad)\n", start_pos * 180.0 / M_PI, start_pos);

    // Continuous oscillation: +45° ↔ start (Ctrl+C to stop)
    float pos_a = start_pos;
    float pos_b = start_pos + (M_PI / 4.0);  // +45°
    printf("  Oscillation: %.2f° ↔ %.2f°\n", pos_a * 180.0 / M_PI, pos_b * 180.0 / M_PI);
    printf("  ⚠ 모터가 계속 왕복합니다! (Ctrl+C로 중단)\n\n");

    printf("[5] Running (Ctrl+C to stop)...\n");
    int cycle = 0;
    while (g_running)
    {
        float target = (cycle % 2 == 0) ? pos_b : pos_a;
        writeParam(motor_id, PARAM_LOC_REF, target);

        // Monitor until settled or timeout (3s)
        for (int i = 0; i < 60 && g_running; ++i)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            float pos = 0, vel = 0;
            readParam(motor_id, PARAM_MECH_POS, pos);
            readParam(motor_id, PARAM_MECH_VEL, vel);

            printf("\r  [cycle %d] target=%.1f°  pos=%.1f°  vel=%.2f rad/s  ",
                   cycle + 1,
                   target * 180.0 / M_PI,
                   pos * 180.0 / M_PI,
                   vel);
            fflush(stdout);

            if (std::abs(pos - target) < 2.0 * M_PI / 180.0 && std::abs(vel) < 0.1)
                break;
        }

        cycle++;
        // Small pause at each end
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    printf("\n");

    // ── Return to start ────────────────────────────────────
    printf("\n[6] Returning to start position...\n");
    writeParam(motor_id, PARAM_LOC_REF, start_pos);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    float final_pos = 0;
    readParam(motor_id, PARAM_MECH_POS, final_pos);
    printf("  Final position: %.2f° (started at %.2f°)\n",
           final_pos * 180.0 / M_PI, start_pos * 180.0 / M_PI);

    // ── Cleanup ────────────────────────────────────────────
    printf("\n[7] Disabling motor...\n");
    disableMotor(motor_id, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    g_can.closeDevice();
    printf("[DONE] Test complete.\n");

    return 0;
}
