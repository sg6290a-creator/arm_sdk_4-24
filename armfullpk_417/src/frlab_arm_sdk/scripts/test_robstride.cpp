/**
 * ============================================================================
 * test_robstride — SocketCAN으로 Robstride 모터 1개 테스트
 * ============================================================================
 *
 * 기능:
 *   1) Enable 모터
 *   2) 상태 읽기 (각도, 속도, 토크, 온도)
 *   3) 위치 모드로 목표 각도 왕복 이동
 *   4) Disable 모터
 *
 * 사전 준비:
 *   sudo slcand -o -c -s8 /dev/ttyACM0 can0
 *   sudo ip link set up can0
 *
 * 사용법:
 *   sudo ./test_robstride [motor_id] [can_if]
 *   sudo ./test_robstride           ← 기본: motor_id=1, can0
 *   sudo ./test_robstride 1 can0
 *
 * ============================================================================
 */

#include "arm_sdk/socketcan_device.hpp"
#include "arm_sdk/robstride_protocol.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <cmath>
#include <chrono>
#include <thread>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ════════════════════════════════════════════════════════════════
//  Globals
// ════════════════════════════════════════════════════════════════

static arm_sdk::SocketCANDevice g_can;
static volatile bool g_running = true;

void signalHandler(int) { g_running = false; }

// ════════════════════════════════════════════════════════════════
//  CAN helpers
// ════════════════════════════════════════════════════════════════

static bool canSend(uint32_t arb_id, const uint8_t* data, uint8_t len = 8)
{
    return g_can.sendFrame(arb_id, data, len, true);  // extended CAN
}

static bool canRecv(uint32_t& arb_id, uint8_t* data, uint8_t& len, int timeout_ms = 200)
{
    bool ext = false;
    return g_can.receiveFrame(arb_id, data, len, timeout_ms, &ext);
}

static bool sendAndRecv(uint32_t arb_id, const uint8_t* data,
                        arm_sdk::RobstrideState* state_out = nullptr)
{
    if (!canSend(arb_id, data)) {
        printf("  [ERROR] CAN send failed\n");
        return false;
    }

    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;

    if (!canRecv(rx_id, rx_data, rx_len, 500)) {
        printf("  [ERROR] No response (timeout)\n");
        return false;
    }

    if (state_out) {
        *state_out = arm_sdk::RobstrideProtocol::parseFeedback(rx_id, rx_data);
    }
    return true;
}

static void flushRx()
{
    uint32_t id; uint8_t d[8], l;
    while (g_can.receiveFrame(id, d, l, 1)) {}
}

// ════════════════════════════════════════════════════════════════
//  Motor commands
// ════════════════════════════════════════════════════════════════

static bool enableMotor(uint8_t motor_id, arm_sdk::RobstrideState* out = nullptr)
{
    uint32_t arb_id;
    uint8_t data[8];
    arm_sdk::RobstrideProtocol::buildEnable(arb_id, data, motor_id);
    return sendAndRecv(arb_id, data, out);
}

static bool disableMotor(uint8_t motor_id)
{
    uint32_t arb_id;
    uint8_t data[8];
    arm_sdk::RobstrideProtocol::buildDisable(arb_id, data, motor_id);
    return canSend(arb_id, data);
}

static bool writeParam(uint8_t motor_id, uint16_t param_id, float value)
{
    uint32_t arb_id;
    uint8_t data[8];
    arm_sdk::RobstrideProtocol::buildWriteParam(arb_id, data, motor_id, param_id, value);
    return sendAndRecv(arb_id, data);
}

static bool readParam(uint8_t motor_id, uint16_t param_id, float& value)
{
    uint32_t arb_id;
    uint8_t data[8];
    arm_sdk::RobstrideProtocol::buildReadParam(arb_id, data, motor_id, param_id);

    if (!canSend(arb_id, data)) return false;

    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;

    for (int attempt = 0; attempt < 10; ++attempt) {
        if (!canRecv(rx_id, rx_data, rx_len, 500)) return false;

        uint8_t msg_type = arm_sdk::RobstrideProtocol::getMsgType(rx_id);
        if (msg_type == arm_sdk::RS_MSG_FEEDBACK) continue;

        if (arm_sdk::RobstrideProtocol::parseParamResponse(rx_data, param_id, value))
            return true;
    }
    return false;
}

// ════════════════════════════════════════════════════════════════
//  Main
// ════════════════════════════════════════════════════════════════

int main(int argc, char* argv[])
{
    signal(SIGINT, signalHandler);

    uint8_t motor_id = (argc >= 2) ? static_cast<uint8_t>(atoi(argv[1])) : 1;
    std::string can_if = (argc >= 3) ? argv[2] : "can0";

    printf("╔══════════════════════════════════════════════╗\n");
    printf("║  Robstride Motor Test (SocketCAN)            ║\n");
    printf("╠══════════════════════════════════════════════╣\n");
    printf("║  Motor ID:    %3d                             ║\n", motor_id);
    printf("║  Interface:   %-10s                    ║\n", can_if.c_str());
    printf("╚══════════════════════════════════════════════╝\n\n");

    // ── Step 1: Open SocketCAN ─────────────────────────────
    printf("[1] Opening SocketCAN interface %s...\n", can_if.c_str());
    if (!g_can.open(can_if)) {
        printf("  [FAIL] Cannot open %s\n", can_if.c_str());
        printf("  Run: sudo slcand -o -c -s8 /dev/ttyACM0 can0 && sudo ip link set up can0\n");
        return 1;
    }
    printf("  [OK] %s open\n\n", can_if.c_str());

    flushRx();

    // ── Step 2: Enable motor ───────────────────────────────
    printf("[2] Enabling motor %d...\n", motor_id);
    arm_sdk::RobstrideState fb;
    if (!enableMotor(motor_id, &fb)) {
        printf("  [FAIL] Enable failed. Motor connected? ID correct?\n");
        disableMotor(motor_id);
        return 1;
    }
    printf("  [OK] Motor enabled!\n");
    printf("  Mode: %s  Angle: %.2f°  Vel: %.2f rad/s  Torque: %.2f Nm  Temp: %.1f°C\n",
           arm_sdk::RobstrideProtocol::modeString(fb.mode),
           fb.position_rad * 180.0 / M_PI,
           fb.velocity_rads,
           fb.torque_nm,
           fb.temperature);
    if (fb.error_bits)
        printf("  ⚠ Error bits: 0x%02X\n", fb.error_bits);
    printf("\n");

    // ── Step 3: Read parameters ────────────────────────────
    printf("[3] Reading parameters...\n");
    float val;
    if (readParam(motor_id, arm_sdk::RS_PARAM_RUN_MODE, val))
        printf("  run_mode = %d (%s)\n", (int)val,
               arm_sdk::RobstrideProtocol::runModeString((uint8_t)val));
    if (readParam(motor_id, arm_sdk::RS_PARAM_MECH_POS, val))
        printf("  mech_pos = %.4f rad (%.2f°)\n", val, val * 180.0 / M_PI);
    if (readParam(motor_id, arm_sdk::RS_PARAM_MECH_VEL, val))
        printf("  mech_vel = %.4f rad/s\n", val);
    if (readParam(motor_id, arm_sdk::RS_PARAM_VBUS, val))
        printf("  vbus     = %.2f V\n", val);
    printf("\n");

    // ── Step 4: Set position mode and move ─────────────────
    printf("[4] Setting Position Mode...\n");

    disableMotor(motor_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    writeParam(motor_id, arm_sdk::RS_PARAM_RUN_MODE, arm_sdk::RS_MODE_POSITION);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    writeParam(motor_id, arm_sdk::RS_PARAM_LIMIT_SPD, 5.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    enableMotor(motor_id, &fb);
    printf("  Mode after set: %s\n", arm_sdk::RobstrideProtocol::modeString(fb.mode));

    float start_pos = 0;
    readParam(motor_id, arm_sdk::RS_PARAM_MECH_POS, start_pos);
    printf("  Current position: %.2f° (%.4f rad)\n", start_pos * 180.0 / M_PI, start_pos);

    float pos_a = start_pos;
    float pos_b = start_pos + (M_PI / 4.0f);  // +45°
    printf("  Oscillation: %.2f° ↔ %.2f°\n", pos_a * 180.0 / M_PI, pos_b * 180.0 / M_PI);
    printf("  ⚠ 모터가 계속 왕복합니다! (Ctrl+C로 중단)\n\n");

    printf("[5] Running (Ctrl+C to stop)...\n");
    int cycle = 0;
    while (g_running) {
        float target = (cycle % 2 == 0) ? pos_b : pos_a;
        writeParam(motor_id, arm_sdk::RS_PARAM_LOC_REF, target);

        for (int i = 0; i < 60 && g_running; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            float pos = 0, vel_val = 0;
            readParam(motor_id, arm_sdk::RS_PARAM_MECH_POS, pos);
            readParam(motor_id, arm_sdk::RS_PARAM_MECH_VEL, vel_val);

            printf("\r  [cycle %d] target=%.1f°  pos=%.1f°  vel=%.2f rad/s  ",
                   cycle + 1, target * 180.0 / M_PI,
                   pos * 180.0 / M_PI, vel_val);
            fflush(stdout);

            if (std::abs(pos - target) < 2.0f * M_PI / 180.0f && std::abs(vel_val) < 0.1f)
                break;
        }

        cycle++;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    printf("\n");

    // ── Return to start ────────────────────────────────────
    printf("\n[6] Returning to start position...\n");
    writeParam(motor_id, arm_sdk::RS_PARAM_LOC_REF, start_pos);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    float final_pos = 0;
    readParam(motor_id, arm_sdk::RS_PARAM_MECH_POS, final_pos);
    printf("  Final position: %.2f° (started at %.2f°)\n",
           final_pos * 180.0 / M_PI, start_pos * 180.0 / M_PI);

    // ── Cleanup ────────────────────────────────────────────
    printf("\n[7] Disabling motor...\n");
    disableMotor(motor_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    printf("[DONE] Test complete.\n");
    return 0;
}
