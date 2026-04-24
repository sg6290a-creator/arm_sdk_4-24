/**
 * ============================================================================
 * test_comm — RMD + Robstride 통신 확인 테스트 (SocketCAN / CANable v2.0)
 * ============================================================================
 *
 * 모터를 움직이지 않고, 각 모터에 상태 요청을 보내고 응답을 확인만 함.
 *   - RMD:       0x9C (Read Status 2) → 응답 파싱
 *   - Robstride: Enable → Feedback 파싱 → 즉시 Disable
 *                ReadParam (mech_pos, mech_vel, vbus) → 응답 파싱
 *
 * 사전 준비:
 *   sudo slcand -o -c -s8 /dev/ttyACM0 can0
 *   sudo ip link set up can0
 *
 * 사용법:
 *   sudo ./test_comm [rmd_count] [rs_id] [can_if]
 *   sudo ./test_comm              ← 4 RMD + Robstride ID 1, can0
 *   sudo ./test_comm 4 1 can0
 *   sudo ./test_comm 0 1 can0    ← Robstride만
 *   sudo ./test_comm 1 0 can0    ← RMD 1개만 (rs_id=0이면 Robstride 생략)
 *
 * ============================================================================
 */

#include "arm_sdk/socketcan_device.hpp"
#include "arm_sdk/rmd_protocol.hpp"
#include "arm_sdk/robstride_protocol.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <string>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ════════════════════════════════════════════════════════════════
//  Globals
// ════════════════════════════════════════════════════════════════

static arm_sdk::SocketCANDevice g_can;
static std::atomic<bool> g_running{true};

static void signalHandler(int) { g_running = false; }

// ════════════════════════════════════════════════════════════════
//  CAN helpers
// ════════════════════════════════════════════════════════════════

static void flushRx(int timeout_ms = 1)
{
    uint32_t id; uint8_t d[8], l;
    while (g_can.receiveFrame(id, d, l, timeout_ms)) {}
}

static void printHex(const uint8_t* data, uint8_t len)
{
    for (int i = 0; i < len; ++i)
        printf("%02X ", data[i]);
}

// ════════════════════════════════════════════════════════════════
//  RMD 통신 테스트
// ════════════════════════════════════════════════════════════════

static bool testRmdMotor(int motor_id)
{
    printf("  ── RMD Motor (ID=%d) ──\n", motor_id);

    uint8_t cmd[8];
    arm_sdk::RMDProtocol::buildReadStatus2(cmd);
    uint32_t tx_id = arm_sdk::RMDProtocol::txId(motor_id);
    uint32_t expected_rx = arm_sdk::RMDProtocol::rxId(motor_id);

    printf("    TX → ID=0x%03X  Data: ", tx_id);
    printHex(cmd, 8);
    printf("\n");

    if (!g_can.sendFrame(tx_id, cmd, 8, false)) {  // standard CAN
        printf("    [FAIL] sendFrame failed\n");
        return false;
    }

    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;

    auto t0 = std::chrono::steady_clock::now();

    for (int attempt = 0; attempt < 30; ++attempt) {
        bool ext = false;
        if (g_can.receiveFrame(rx_id, rx_data, rx_len, 5, &ext)) {
            auto dt = std::chrono::steady_clock::now() - t0;
            double ms = std::chrono::duration<double, std::milli>(dt).count();

            if (!ext && rx_id == expected_rx) {
                printf("    RX ← ID=0x%03X  Data: ", rx_id);
                printHex(rx_data, rx_len);
                printf(" (%.1f ms)\n", ms);

                arm_sdk::MotorState state = arm_sdk::RMDProtocol::parseStatus2(rx_data, rx_len, 0.32);
                if (state.valid) {
                    printf("    ✓ Position: %.2f° (%.4f rad)\n",
                           arm_sdk::RMDProtocol::radToDeg(state.position_rad), state.position_rad);
                    printf("    ✓ Velocity: %.2f dps (%.4f rad/s)\n",
                           arm_sdk::RMDProtocol::radSToDps(state.velocity_rads), state.velocity_rads);
                    printf("    ✓ Effort:   %.3f Nm\n", state.effort_nm);
                    printf("    ✓ Temp:     %d °C\n", (int)state.temperature);
                    printf("    [OK] RMD ID=%d responded\n\n", motor_id);
                    return true;
                } else {
                    printf("    [WARN] Response received but parse failed\n\n");
                    return false;
                }
            } else {
                printf("    (ignored frame: ext=%d id=0x%X)\n", (int)ext, rx_id);
            }
        }
    }

    printf("    [FAIL] No response (timeout)\n\n");
    return false;
}

// ════════════════════════════════════════════════════════════════
//  Robstride 통신 테스트
// ════════════════════════════════════════════════════════════════

static bool testRobstrideMotor(int motor_id)
{
    printf("  ── Robstride Motor (ID=%d) ──\n", motor_id);

    // --- Test 1: Enable → get feedback → Disable ---
    printf("    [Test 1] Enable → Feedback → Disable\n");

    uint32_t arb_id;
    uint8_t data[8];
    arm_sdk::RobstrideProtocol::buildEnable(arb_id, data, static_cast<uint8_t>(motor_id));

    printf("    TX → ArbID=0x%08X (Enable)  Data: ", arb_id);
    printHex(data, 8);
    printf("\n");

    if (!g_can.sendFrame(arb_id, data, 8, true)) {  // extended CAN
        printf("    [FAIL] sendFrame failed\n");
        return false;
    }

    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;

    auto t0 = std::chrono::steady_clock::now();
    bool got_feedback = false;

    for (int attempt = 0; attempt < 50; ++attempt) {
        bool ext = false;
        if (g_can.receiveFrame(rx_id, rx_data, rx_len, 10, &ext)) {
            auto dt = std::chrono::steady_clock::now() - t0;
            double ms = std::chrono::duration<double, std::milli>(dt).count();

            if (ext) {
                printf("    RX ← ArbID=0x%08X  Data: ", rx_id);
                printHex(rx_data, rx_len);
                printf(" (%.1f ms)\n", ms);

                arm_sdk::RobstrideState state = arm_sdk::RobstrideProtocol::parseFeedback(rx_id, rx_data);
                if (state.valid) {
                    printf("    ✓ Motor ID:  %d\n", state.motor_id);
                    printf("    ✓ Mode:      %s (%d)\n",
                           arm_sdk::RobstrideProtocol::modeString(state.mode), state.mode);
                    printf("    ✓ Position:  %.2f° (%.4f rad)\n",
                           state.position_rad * 180.0 / M_PI, state.position_rad);
                    printf("    ✓ Velocity:  %.4f rad/s\n", state.velocity_rads);
                    printf("    ✓ Torque:    %.3f Nm\n", state.torque_nm);
                    printf("    ✓ Temp:      %.1f °C\n", state.temperature);
                    if (state.error_bits)
                        printf("    ⚠ Errors:   0x%02X\n", state.error_bits);
                    got_feedback = true;
                }
                break;
            } else {
                printf("    (ignored standard frame: id=0x%X)\n", rx_id);
            }
        }
    }

    if (!got_feedback) {
        printf("    [FAIL] No feedback response\n\n");
        return false;
    }

    // Immediately disable
    arm_sdk::RobstrideProtocol::buildDisable(arb_id, data, static_cast<uint8_t>(motor_id));
    g_can.sendFrame(arb_id, data, 8, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    flushRx();

    printf("    [OK] Enable/Feedback/Disable OK\n\n");

    // --- Test 2: Read Parameters (no movement) ---
    printf("    [Test 2] Read Parameters\n");

    struct ParamTest {
        uint16_t id;
        const char* name;
        const char* unit;
    };
    ParamTest params[] = {
        {arm_sdk::RS_PARAM_RUN_MODE,  "run_mode",  ""},
        {arm_sdk::RS_PARAM_MECH_POS,  "mech_pos",  "rad"},
        {arm_sdk::RS_PARAM_MECH_VEL,  "mech_vel",  "rad/s"},
        {arm_sdk::RS_PARAM_VBUS,      "vbus",      "V"},
        {arm_sdk::RS_PARAM_LIMIT_SPD, "limit_spd", "rad/s"},
    };

    // Enable to read params
    arm_sdk::RobstrideProtocol::buildEnable(arb_id, data, static_cast<uint8_t>(motor_id));
    g_can.sendFrame(arb_id, data, 8, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    flushRx();

    int param_ok = 0;
    for (auto& p : params) {
        arm_sdk::RobstrideProtocol::buildReadParam(arb_id, data,
            static_cast<uint8_t>(motor_id), p.id);

        printf("    TX → ReadParam(0x%04X='%s')  ArbID=0x%08X\n", p.id, p.name, arb_id);

        if (!g_can.sendFrame(arb_id, data, 8, true)) {
            printf("      [FAIL] send failed\n");
            continue;
        }

        bool ext = false;
        t0 = std::chrono::steady_clock::now();
        bool found = false;

        for (int a = 0; a < 30; ++a) {
            if (g_can.receiveFrame(rx_id, rx_data, rx_len, 10, &ext)) {
                if (ext) {
                    auto dt = std::chrono::steady_clock::now() - t0;
                    double ms = std::chrono::duration<double, std::milli>(dt).count();

                    printf("    RX ← ArbID=0x%08X  Data: ", rx_id);
                    printHex(rx_data, rx_len);
                    printf(" (%.1f ms)\n", ms);

                    float value = 0;
                    if (arm_sdk::RobstrideProtocol::parseParamResponse(rx_data, p.id, value)) {
                        if (p.id == arm_sdk::RS_PARAM_RUN_MODE) {
                            printf("    ✓ %s = %d (%s)\n", p.name, (int)value,
                                   arm_sdk::RobstrideProtocol::runModeString((uint8_t)value));
                        } else if (p.id == arm_sdk::RS_PARAM_MECH_POS) {
                            printf("    ✓ %s = %.4f %s (%.2f°)\n",
                                   p.name, value, p.unit, value * 180.0f / M_PI);
                        } else {
                            printf("    ✓ %s = %.4f %s\n", p.name, value, p.unit);
                        }
                        param_ok++;
                        found = true;
                    } else {
                        printf("    [WARN] param_id mismatch in response\n");
                    }
                    break;
                }
            }
        }
        if (!found) {
            printf("      [FAIL] no response\n");
        }
    }

    // Disable
    arm_sdk::RobstrideProtocol::buildDisable(arb_id, data, static_cast<uint8_t>(motor_id));
    g_can.sendFrame(arb_id, data, 8, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    printf("    [OK] %d/%zu params read\n\n", param_ok, sizeof(params)/sizeof(params[0]));
    return got_feedback;
}

// ════════════════════════════════════════════════════════════════
//  Main
// ════════════════════════════════════════════════════════════════

int main(int argc, char* argv[])
{
    int rmd_count   = (argc >= 2) ? std::atoi(argv[1]) : 4;
    int rs_motor_id = (argc >= 3) ? std::atoi(argv[2]) : 1;
    std::string can_if = (argc >= 4) ? argv[3] : "can0";

    bool test_robstride = (rs_motor_id > 0);

    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║  Communication Test (No Movement)  SocketCAN          ║\n");
    printf("╠═══════════════════════════════════════════════════════╣\n");
    printf("║  RMD motors:       %d (ID 1-%d)                       ║\n", rmd_count, rmd_count);
    if (test_robstride)
        printf("║  Robstride motor:  ID %d                              ║\n", rs_motor_id);
    else
        printf("║  Robstride motor:  none                               ║\n");
    printf("║  Interface:        %-10s                        ║\n", can_if.c_str());
    printf("╚═══════════════════════════════════════════════════════╝\n\n");

    // ── Open SocketCAN ─────────────────────────────────────
    printf("[1] Opening SocketCAN interface %s...\n", can_if.c_str());
    if (!g_can.open(can_if)) {
        printf("  [FAIL] Cannot open %s\n", can_if.c_str());
        printf("  Run: sudo slcand -o -c -s8 /dev/ttyACM0 can0 && sudo ip link set up can0\n");
        return 1;
    }
    printf("  [OK] %s open\n\n", can_if.c_str());

    flushRx();

    // ── Test each motor ────────────────────────────────────
    int pass = 0, fail = 0;

    printf("[2] Testing motors...\n\n");

    for (int i = 1; i <= rmd_count; ++i) {
        if (testRmdMotor(i)) pass++;
        else fail++;
    }

    if (test_robstride) {
        if (testRobstrideMotor(rs_motor_id)) pass++;
        else fail++;
    }

    // ── Summary ────────────────────────────────────────────
    printf("═══════════════════════════════════════════════════════\n");
    printf("  Result:  %d PASS  /  %d FAIL  /  %d total\n", pass, fail, pass + fail);
    printf("═══════════════════════════════════════════════════════\n");

    if (fail > 0) return 1;

    // ══════════════════════════════════════════════════════════
    //  [3] Continuous state reading loop
    // ══════════════════════════════════════════════════════════
    printf("\n[3] Continuous state reading (Ctrl+C to stop)\n");
    printf("    RMD: %d motors   Robstride: %s\n\n", rmd_count,
           test_robstride ? "yes" : "no");

    signal(SIGINT, signalHandler);

    // Enable Robstride once before loop
    if (test_robstride) {
        uint32_t arb_id;
        uint8_t data[8];
        arm_sdk::RobstrideProtocol::buildEnable(arb_id, data, static_cast<uint8_t>(rs_motor_id));
        g_can.sendFrame(arb_id, data, 8, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        flushRx();
    }

    // Pre-build TX frames
    const int total_motors = rmd_count + (test_robstride ? 1 : 0);
    arm_sdk::CANFrame tx_frames[8];
    int tx_count = 0;

    for (int i = 1; i <= rmd_count; ++i) {
        uint8_t cmd[8];
        arm_sdk::RMDProtocol::buildReadStatus2(cmd);
        tx_frames[tx_count].id       = arm_sdk::RMDProtocol::txId(i);
        tx_frames[tx_count].extended = false;
        tx_frames[tx_count].data_len = 8;
        memcpy(tx_frames[tx_count].data, cmd, 8);
        tx_count++;
    }

    if (test_robstride) {
        uint32_t arb_id;
        uint8_t data[8];
        arm_sdk::RobstrideProtocol::buildEnable(arb_id, data, static_cast<uint8_t>(rs_motor_id));
        tx_frames[tx_count].id       = arb_id;
        tx_frames[tx_count].extended = true;
        tx_frames[tx_count].data_len = 8;
        memcpy(tx_frames[tx_count].data, data, 8);
        tx_count++;
    }

    // State storage
    struct MotorReading {
        bool valid = false;
        double deg = 0, dps = 0, torque = 0;
        int temp = 0;
        bool error = false;
    };
    MotorReading readings[8];

    char buf[4096];
    int cycle = 0;
    double avg_hz = 0;
    auto display_time = std::chrono::steady_clock::now();
    setvbuf(stdout, nullptr, _IOFBF, 8192);

    uint32_t rmd_rx_ids[8];
    for (int i = 0; i < rmd_count; ++i)
        rmd_rx_ids[i] = arm_sdk::RMDProtocol::rxId(i + 1);

    while (g_running) {
        cycle++;
        auto cycle_start = std::chrono::steady_clock::now();

        for (int i = 0; i < total_motors; ++i) readings[i].valid = false;

        // Batch send
        g_can.sendFrames(tx_frames, tx_count);

        // Receive with deadline
        arm_sdk::CANFrame rx_frames[32];
        int total_rx = 0;
        int got_count = 0;

        auto deadline = cycle_start + std::chrono::microseconds(2000);
        while (got_count < total_motors && std::chrono::steady_clock::now() < deadline) {
            int n = g_can.receiveFrames(&rx_frames[total_rx], 16 - total_rx, 0);
            if (n > 0) {
                for (int j = 0; j < n; ++j) {
                    auto& f = rx_frames[total_rx + j];
                    if (!f.extended) {
                        for (int m = 0; m < rmd_count; ++m) {
                            if (f.id == rmd_rx_ids[m] && !readings[m].valid) {
                                arm_sdk::MotorState st = arm_sdk::RMDProtocol::parseStatus2(f.data, f.data_len, 0.32);
                                if (st.valid) {
                                    readings[m].valid  = true;
                                    readings[m].deg    = arm_sdk::RMDProtocol::radToDeg(st.position_rad);
                                    readings[m].dps    = arm_sdk::RMDProtocol::radSToDps(st.velocity_rads);
                                    readings[m].torque = st.effort_nm;
                                    readings[m].temp   = (int)st.temperature;
                                    got_count++;
                                }
                                break;
                            }
                        }
                    } else if (test_robstride && !readings[rmd_count].valid) {
                        arm_sdk::RobstrideState st = arm_sdk::RobstrideProtocol::parseFeedback(f.id, f.data);
                        if (st.valid) {
                            int idx = rmd_count;
                            readings[idx].valid  = true;
                            readings[idx].deg    = st.position_rad * 180.0 / M_PI;
                            readings[idx].dps    = st.velocity_rads * 180.0 / M_PI;
                            readings[idx].torque = st.torque_nm;
                            readings[idx].temp   = static_cast<int>(st.temperature);
                            readings[idx].error  = (st.error_bits != 0);
                            got_count++;
                        }
                    }
                }
                total_rx += n;
            } else {
                usleep(10);
            }
        }

        auto cycle_end = std::chrono::steady_clock::now();
        double cycle_ms = std::chrono::duration<double, std::milli>(cycle_end - cycle_start).count();
        double hz = (cycle_ms > 0.0) ? 1000.0 / cycle_ms : 0.0;
        avg_hz = (cycle == 1) ? hz : avg_hz * 0.95 + hz * 0.05;

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double, std::milli>(now - display_time).count() > 100.0) {
            display_time = now;
            int pos = 0;
            pos += snprintf(buf + pos, sizeof(buf) - pos, "\033[2J\033[H");
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                "Motor State [Cycle %d]  %.2f ms  avg %.0f Hz   Ctrl+C stop\n"
                "%-8s %10s %10s %10s %6s %s\n",
                cycle, cycle_ms, avg_hz,
                "Motor", "Pos(°)", "Vel(°/s)", "Torq(Nm)", "Temp", "St");

            for (int m = 0; m < rmd_count; ++m) {
                if (readings[m].valid)
                    pos += snprintf(buf + pos, sizeof(buf) - pos,
                        "RMD %-3d  %10.2f %10.2f %10.3f %4d°C  OK\n",
                        m + 1, readings[m].deg, readings[m].dps, readings[m].torque, readings[m].temp);
                else
                    pos += snprintf(buf + pos, sizeof(buf) - pos,
                        "RMD %-3d  %10s %10s %10s %6s  --\n", m + 1, "---", "---", "---", "---");
            }
            if (test_robstride) {
                int idx = rmd_count;
                if (readings[idx].valid)
                    pos += snprintf(buf + pos, sizeof(buf) - pos,
                        "RS  %-3d  %10.2f %10.2f %10.3f %4d°C  %s\n",
                        rs_motor_id, readings[idx].deg, readings[idx].dps, readings[idx].torque,
                        readings[idx].temp, readings[idx].error ? "ER" : "OK");
                else
                    pos += snprintf(buf + pos, sizeof(buf) - pos,
                        "RS  %-3d  %10s %10s %10s %6s  --\n", rs_motor_id, "---", "---", "---", "---");
            }

            fwrite(buf, 1, pos, stdout);
            fflush(stdout);
        }
    }

    // Disable Robstride on exit
    if (test_robstride) {
        uint32_t arb_id;
        uint8_t data[8];
        arm_sdk::RobstrideProtocol::buildDisable(arb_id, data, static_cast<uint8_t>(rs_motor_id));
        g_can.sendFrame(arb_id, data, 8, true);
    }

    printf("\n[Stopped]\n");
    return 0;
}
