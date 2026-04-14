/**
 * ============================================================================
 * test_comm — RMD + Robstride 통신 확인 테스트 (움직임 없음)
 * ============================================================================
 *
 * 모터를 움직이지 않고, 각 모터에 상태 요청을 보내고 응답을 확인만 함.
 *   - RMD:       0x9C (Read Status 2) → 응답 파싱
 *   - Robstride: Enable → Feedback 파싱 → 즉시 Disable
 *                ReadParam (mech_pos, mech_vel, vbus) → 응답 파싱
 *
 * 사용법:
 *   sudo ./test_comm [rmd_count] [rs_id] [can_channel]
 *   sudo ./test_comm              ← 4 RMD + Robstride ID 1, ch0
 *   sudo ./test_comm 4 1 0
 *   sudo ./test_comm 0 1 0        ← Robstride만
 *   sudo ./test_comm 1 0 0        ← RMD 1개만 (rs_id=0이면 Robstride 생략)
 *
 * ============================================================================
 */

#include "arm_sdk/usbcan_device.hpp"
#include "arm_sdk/rmd_protocol.hpp"
#include "arm_sdk/robstride_protocol.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ════════════════════════════════════════════════════════════════
//  Globals
// ════════════════════════════════════════════════════════════════

static arm_sdk::USBCANDevice g_can;
static uint32_t g_channel = 0;
static std::atomic<bool> g_running{true};

static void signalHandler(int) { g_running = false; }

// ════════════════════════════════════════════════════════════════
//  CAN helpers
// ════════════════════════════════════════════════════════════════

/// Flush all pending CAN frames
static void flushRx(int timeout_ms = 1)
{
    uint32_t id; uint8_t d[8], l;
    while (g_can.receiveFrame(g_channel, id, d, l, timeout_ms)) {}
}

/// Print 8-byte data as hex
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

    // Build 0x9C Read Status 2
    uint8_t cmd[8];
    arm_sdk::RMDProtocol::buildReadStatus2(cmd);
    uint32_t tx_id = arm_sdk::RMDProtocol::txId(motor_id);
    uint32_t expected_rx = arm_sdk::RMDProtocol::rxId(motor_id);

    printf("    TX → ID=0x%03X  Data: ", tx_id);
    printHex(cmd, 8);
    printf("\n");

    // Send (standard CAN, extern_flag=0)
    if (!g_can.sendFrame(g_channel, tx_id, cmd, 8, 0)) {
        printf("    [FAIL] sendFrame failed\n");
        return false;
    }

    // Receive
    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;

    auto t0 = std::chrono::steady_clock::now();

    for (int attempt = 0; attempt < 30; ++attempt) {
        uint8_t ext = 0;
        if (g_can.receiveFrame(g_channel, rx_id, rx_data, rx_len, 5, &ext)) {
            auto dt = std::chrono::steady_clock::now() - t0;
            double ms = std::chrono::duration<double, std::milli>(dt).count();

            if (ext == 0 && rx_id == expected_rx) {
                printf("    RX ← ID=0x%03X  Data: ", rx_id);
                printHex(rx_data, rx_len);
                printf(" (%.1f ms)\n", ms);

                // Parse
                arm_sdk::MotorState state = arm_sdk::RMDProtocol::parseStatus2(rx_data, rx_len, 0.32);
                if (state.valid) {
                    printf("    ✓ Position: %.2f° (%.4f rad)\n",
                           arm_sdk::RMDProtocol::radToDeg(state.position_rad), state.position_rad);
                    printf("    ✓ Velocity: %.2f dps (%.4f rad/s)\n",
                           arm_sdk::RMDProtocol::radSToDps(state.velocity_rads), state.velocity_rads);
                    printf("    ✓ Effort:   %.3f Nm\n", state.effort_nm);
                    printf("    ✓ Temp:     %d °C\n", state.temperature);
                    printf("    [OK] RMD ID=%d responded\n\n", motor_id);
                    return true;
                } else {
                    printf("    [WARN] Response received but parse failed\n\n");
                    return false;
                }
            } else {
                printf("    (ignored frame: ext=%d id=0x%X)\n", ext, rx_id);
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

    // Send (extended CAN, extern_flag=1)
    if (!g_can.sendFrame(g_channel, arb_id, data, 8, 1)) {
        printf("    [FAIL] sendFrame failed\n");
        return false;
    }

    // Receive feedback
    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;

    auto t0 = std::chrono::steady_clock::now();
    bool got_feedback = false;

    for (int attempt = 0; attempt < 50; ++attempt) {
        uint8_t ext = 0;
        if (g_can.receiveFrame(g_channel, rx_id, rx_data, rx_len, 10, &ext)) {
            auto dt = std::chrono::steady_clock::now() - t0;
            double ms = std::chrono::duration<double, std::milli>(dt).count();

            if (ext == 1) {
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
    g_can.sendFrame(g_channel, arb_id, data, 8, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    flushRx();

    printf("    [OK] Enable/Feedback/Disable OK\n\n");

    // --- Test 2: Read Parameters (no movement) ---
    printf("    [Test 2] Read Parameters\n");

    struct ParamTest {
        uint16_t id;
        const char* name;
        const char* unit;
        float scale;  // for display
    };
    ParamTest params[] = {
        {arm_sdk::RS_PARAM_RUN_MODE,  "run_mode",  "",     1.0f},
        {arm_sdk::RS_PARAM_MECH_POS,  "mech_pos",  "rad",  1.0f},
        {arm_sdk::RS_PARAM_MECH_VEL,  "mech_vel",  "rad/s",1.0f},
        {arm_sdk::RS_PARAM_VBUS,      "vbus",      "V",    1.0f},
        {arm_sdk::RS_PARAM_LIMIT_SPD, "limit_spd", "rad/s",1.0f},
    };

    // Need to enable to read params
    arm_sdk::RobstrideProtocol::buildEnable(arb_id, data, static_cast<uint8_t>(motor_id));
    g_can.sendFrame(g_channel, arb_id, data, 8, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    flushRx();

    int param_ok = 0;
    for (auto& p : params) {
        arm_sdk::RobstrideProtocol::buildReadParam(arb_id, data,
            static_cast<uint8_t>(motor_id), p.id);

        printf("    TX → ReadParam(0x%04X='%s')  ArbID=0x%08X\n", p.id, p.name, arb_id);

        if (!g_can.sendFrame(g_channel, arb_id, data, 8, 1)) {
            printf("      [FAIL] send failed\n");
            continue;
        }

        uint8_t ext = 0;
        t0 = std::chrono::steady_clock::now();
        bool found = false;

        for (int a = 0; a < 30; ++a) {
            if (g_can.receiveFrame(g_channel, rx_id, rx_data, rx_len, 10, &ext)) {
                if (ext == 1) {
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
    g_can.sendFrame(g_channel, arb_id, data, 8, 1);
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
    g_channel       = (argc >= 4) ? static_cast<uint32_t>(std::atoi(argv[3])) : 0;

    bool test_robstride = (rs_motor_id > 0);

    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║  Communication Test (No Movement)                     ║\n");
    printf("╠═══════════════════════════════════════════════════════╣\n");
    printf("║  RMD motors:       %d (ID 1-%d)                       ║\n", rmd_count, rmd_count);
    if (test_robstride)
        printf("║  Robstride motor:  ID %d                              ║\n", rs_motor_id);
    else
        printf("║  Robstride motor:  none                               ║\n");
    printf("║  CAN channel:      %d                                 ║\n", g_channel);
    printf("╚═══════════════════════════════════════════════════════╝\n\n");

    // ── Open USBCAN ────────────────────────────────────────
    printf("[1] Opening USBCAN...\n");
    if (!g_can.loadLibrary("libusbcan.so")) {
        printf("  [FAIL] Cannot load libusbcan.so\n");
        return 1;
    }
    if (!g_can.openDevice(arm_sdk::USBCAN_II, 0)) {
        printf("  [FAIL] Cannot open device\n");
        return 1;
    }
    uint16_t timing = arm_sdk::USBCANDevice::baudrateToTiming(1000000);
    if (!g_can.initChannel(g_channel, timing) || !g_can.startChannel(g_channel)) {
        printf("  [FAIL] Cannot init/start CAN ch%d\n", g_channel);
        return 1;
    }
    printf("  [OK] USBCAN ready (1Mbps, ch%d)\n\n", g_channel);

    flushRx();

    // ── Test each motor ────────────────────────────────────
    int pass = 0, fail = 0;

    printf("[2] Testing motors...\n\n");

    for (int i = 1; i <= rmd_count; ++i) {
        if (testRmdMotor(i))
            pass++;
        else
            fail++;
    }

    if (test_robstride) {
        if (testRobstrideMotor(rs_motor_id))
            pass++;
        else
            fail++;
    }

    // ── Summary ────────────────────────────────────────────
    printf("═══════════════════════════════════════════════════════\n");
    printf("  Result:  %d PASS  /  %d FAIL  /  %d total\n", pass, fail, pass + fail);
    printf("═══════════════════════════════════════════════════════\n");

    if (fail > 0) {
        g_can.closeDevice();
        return 1;
    }

    // ══════════════════════════════════════════════════════════
    //  [3] Continuous state reading loop — BATCH optimized
    // ══════════════════════════════════════════════════════════
    printf("\n[3] Continuous state reading (Ctrl+C to stop)\n");
    printf("    RMD: %d motors   Robstride: %s\n\n", rmd_count,
           test_robstride ? "yes" : "no");

    signal(SIGINT, signalHandler);

    // Enable Robstride once before loop (stays enabled)
    if (test_robstride) {
        uint32_t arb_id;
        uint8_t data[8];
        arm_sdk::RobstrideProtocol::buildEnable(arb_id, data, static_cast<uint8_t>(rs_motor_id));
        g_can.sendFrame(g_channel, arb_id, data, 8, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        flushRx();
    }

    // Pre-build TX frames (never changes)
    const int total_motors = rmd_count + (test_robstride ? 1 : 0);
    arm_sdk::CANFrame tx_frames[8];
    int tx_count = 0;

    // RMD read commands
    for (int i = 1; i <= rmd_count; ++i) {
        memset(&tx_frames[tx_count], 0, sizeof(arm_sdk::CANFrame));
        uint8_t cmd[8];
        arm_sdk::RMDProtocol::buildReadStatus2(cmd);
        tx_frames[tx_count].id = arm_sdk::RMDProtocol::txId(i);
        tx_frames[tx_count].extern_flag = 0;
        tx_frames[tx_count].data_len = 8;
        memcpy(tx_frames[tx_count].data, cmd, 8);
        tx_count++;
    }

    // Robstride enable (for feedback)
    int rs_tx_idx = -1;
    if (test_robstride) {
        rs_tx_idx = tx_count;
        memset(&tx_frames[tx_count], 0, sizeof(arm_sdk::CANFrame));
        uint32_t arb_id;
        uint8_t data[8];
        arm_sdk::RobstrideProtocol::buildEnable(arb_id, data, static_cast<uint8_t>(rs_motor_id));
        tx_frames[tx_count].id = arb_id;
        tx_frames[tx_count].extern_flag = 1;
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

    // Output buffer & display throttle
    char buf[4096];
    int cycle = 0;
    double avg_hz = 0;
    auto display_time = std::chrono::steady_clock::now();
    setvbuf(stdout, nullptr, _IOFBF, 8192);  // fully buffered stdout

    // Pre-compute expected RX IDs
    uint32_t rmd_rx_ids[8];
    for (int i = 0; i < rmd_count; ++i)
        rmd_rx_ids[i] = arm_sdk::RMDProtocol::rxId(i + 1);

    while (g_running) {
        cycle++;
        auto cycle_start = std::chrono::steady_clock::now();

        // Reset readings
        for (int i = 0; i < total_motors; ++i) readings[i].valid = false;

        // ── BATCH SEND all frames in one USB call ──
        g_can.sendFrames(g_channel, tx_frames, tx_count);

        // ── SPIN-POLL RECEIVE (timeout=0, no kernel sleep) ──
        arm_sdk::CANFrame rx_frames[32];
        int total_rx = 0;
        int got_count = 0;

        // Spin until all responses or ~2ms deadline
        auto deadline = cycle_start + std::chrono::microseconds(2000);
        while (got_count < total_motors && std::chrono::steady_clock::now() < deadline) {
            int n = g_can.receiveFrames(g_channel, &rx_frames[total_rx],
                                         16 - total_rx, 0);  // timeout=0!
            if (n > 0) {
                // Quick-parse inline to count matched
                for (int j = 0; j < n; ++j) {
                    auto& f = rx_frames[total_rx + j];
                    if (f.extern_flag == 0) {
                        for (int m = 0; m < rmd_count; ++m) {
                            if (f.id == rmd_rx_ids[m] && !readings[m].valid) {
                                arm_sdk::MotorState st = arm_sdk::RMDProtocol::parseStatus2(f.data, f.data_len, 0.32);
                                if (st.valid) {
                                    readings[m].valid = true;
                                    readings[m].deg = arm_sdk::RMDProtocol::radToDeg(st.position_rad);
                                    readings[m].dps = arm_sdk::RMDProtocol::radSToDps(st.velocity_rads);
                                    readings[m].torque = st.effort_nm;
                                    readings[m].temp = st.temperature;
                                    got_count++;
                                }
                                break;
                            }
                        }
                    } else if (test_robstride && !readings[rmd_count].valid) {
                        arm_sdk::RobstrideState st = arm_sdk::RobstrideProtocol::parseFeedback(f.id, f.data);
                        if (st.valid) {
                            int idx = rmd_count;
                            readings[idx].valid = true;
                            readings[idx].deg = st.position_rad * 180.0 / M_PI;
                            readings[idx].dps = st.velocity_rads * 180.0 / M_PI;
                            readings[idx].torque = st.torque_nm;
                            readings[idx].temp = static_cast<int>(st.temperature);
                            readings[idx].error = (st.error_bits != 0);
                            got_count++;
                        }
                    }
                }
                total_rx += n;
            } else {
                usleep(10);  // yield CPU briefly when no data yet
            }
        }

        auto cycle_end = std::chrono::steady_clock::now();
        double cycle_ms = std::chrono::duration<double, std::milli>(cycle_end - cycle_start).count();
        double hz = (cycle_ms > 0.0) ? 1000.0 / cycle_ms : 0.0;

        // Exponential moving average
        avg_hz = (cycle == 1) ? hz : avg_hz * 0.95 + hz * 0.05;

        // ── Display throttle: refresh every ~100ms (no output = no syscall overhead) ──
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
        g_can.sendFrame(g_channel, arb_id, data, 8, 1);
    }

    printf("\n[Stopped] Closing USBCAN...\n");
    g_can.closeDevice();

    return 0;
}
