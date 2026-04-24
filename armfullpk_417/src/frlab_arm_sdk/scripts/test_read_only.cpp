/**
 * ============================================================================
 * test_read_only — 6DOF 모터 통신/홀드 테스트
 * ============================================================================
 *
 * 기본 동작:
 *   1) 소켓CAN 인터페이스 상태 확인 (UP / bitrate)
 *   2) RMD (MyActuator X4-36) x4: 0xA1 토크 명령 (iq=0) → 응답 확인
 *   3) Robstride03 x2: Enable → Feedback → 즉시 Disable
 *   4) 필요 시 현재 위치 홀드 모니터링 (Ctrl+C 종료)
 *
 * --comm-only 옵션:
 *   Step 4 결과 요약까지만 수행하고 종료합니다.
 *   위치 홀드 Step 5로 들어가지 않으므로 송수신 확인용으로 안전합니다.
 *
 * 하드코딩 구성:
 *   Joint 1: Robstride03   ID=1    (Extended CAN 29-bit)
 *   Joint 2: Robstride03   ID=127  (Extended CAN 29-bit)
 *   Joint 3: MyActuator    ID=1    (Standard CAN 11-bit)
 *   Joint 4: MyActuator    ID=2    (Standard CAN 11-bit)
 *   Joint 5: MyActuator    ID=3    (Standard CAN 11-bit)
 *   Joint 6: MyActuator    ID=4    (Standard CAN 11-bit)
 *
 * 사전 준비:
 *   sudo slcand -o -c -s8 /dev/ttyACM0 can0
 *   sudo ip link set up can0
 *
 * 사용법:
 *   sudo ./test_read_only [can_if] [period_ms] [--comm-only]
 *   예) sudo ./test_read_only can0 20                    ← 20ms=50Hz, 전체 위치 홀드
 *       sudo ./test_read_only can0 5                     ← 5ms=200Hz, 전체 위치 홀드
 *       sudo ./test_read_only can0 20 --comm-only        ← 송수신 확인만 하고 종료
 *
 * ============================================================================
 */

#include "arm_sdk/socketcan_device.hpp"
#include "arm_sdk/rmd_protocol.hpp"
#include "arm_sdk/robstride_protocol.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <cmath>
#include <chrono>
#include <thread>
#include <string>
#include <atomic>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ════════════════════════════════════════════════════════════════
//  하드코딩 모터 설정
// ════════════════════════════════════════════════════════════════

// Robstride03 모터
static constexpr int RS_MOTOR_1_ID  = 1;
static constexpr int RS_MOTOR_2_ID  = 127;

// MyActuator RMD (X4-36) 모터
static constexpr int RMD_MOTOR_1_ID = 1;
static constexpr int RMD_MOTOR_2_ID = 2;
static constexpr int RMD_MOTOR_3_ID = 3;
static constexpr int RMD_MOTOR_4_ID = 4;

// RMD 토크 상수
static constexpr double RMD_TORQUE_CONST = 0.32;  // Nm/A

// RMD 토크 명령 (0xA1)
static constexpr uint8_t RMD_CMD_TORQUE_CTRL = 0xA1;

// 제어 루프 설정
static int g_loop_period_ms = 20;  // 기본 20ms = 50Hz (인자로 변경 가능)

// ════════════════════════════════════════════════════════════════
//  Globals
// ════════════════════════════════════════════════════════════════

static arm_sdk::SocketCANDevice g_can;
static std::atomic<bool> g_running{true};
static bool g_comm_only = false;

static void signalHandler(int) { g_running = false; }

// ════════════════════════════════════════════════════════════════
//  모터 상태 구조체
// ════════════════════════════════════════════════════════════════

struct MotorResult {
    bool   ok       = false;
    double pos_deg  = 0.0;
    double vel_dps  = 0.0;
    double torque   = 0.0;
    double temp_c   = 0.0;
    uint8_t error   = 0;
    // 제어 입력 (TX 명령)
    double cmd_torque = 0.0;   // 명령 토크 (Nm)
    double cmd_vel    = 0.0;   // 명령 속도
    double cmd_pos    = 0.0;   // 명령 위치
    const char* cmd_type = "iq=0"; // 명령 타입
};

// ════════════════════════════════════════════════════════════════
//  SocketCAN 상태 확인
// ════════════════════════════════════════════════════════════════

static bool checkSocketCAN(const std::string& iface)
{
    printf("  [1] 인터페이스 존재 확인: /sys/class/net/%s\n", iface.c_str());

    std::string path = "/sys/class/net/" + iface + "/operstate";
    FILE* f = fopen(path.c_str(), "r");
    if (!f) {
        printf("      [FAIL] '%s' 인터페이스가 없습니다!\n", iface.c_str());
        printf("      → sudo slcand -o -c -s8 /dev/ttyACM0 %s\n", iface.c_str());
        return false;
    }

    char state[64] = {0};
    if (fgets(state, sizeof(state), f)) {
        for (int i = 0; state[i]; ++i)
            if (state[i] == '\n' || state[i] == '\r') state[i] = '\0';
    }
    fclose(f);

    printf("      operstate = \"%s\"\n", state);

    if (strcmp(state, "up") == 0 || strcmp(state, "unknown") == 0) {
        printf("      [OK] 인터페이스 활성 상태\n");
    } else {
        printf("      [FAIL] 인터페이스 DOWN\n");
        printf("      → sudo ip link set up %s\n", iface.c_str());
        return false;
    }

    // CAN 타입 확인
    std::string type_path = "/sys/class/net/" + iface + "/type";
    f = fopen(type_path.c_str(), "r");
    if (f) {
        int type_val = 0;
        if (fscanf(f, "%d", &type_val) == 1)
            printf("  [2] 네트워크 타입: %d %s\n", type_val,
                   (type_val == 280) ? "(CAN ✓)" : "(NOT CAN ⚠)");
        fclose(f);
    }

    // bitrate 확인
    std::string br_path = "/sys/class/net/" + iface + "/can_bittiming/bitrate";
    f = fopen(br_path.c_str(), "r");
    if (f) {
        int bitrate = 0;
        if (fscanf(f, "%d", &bitrate) == 1)
            printf("  [3] Bitrate: %d bps (%s)\n", bitrate,
                   (bitrate == 1000000) ? "1Mbps ✓" :
                   (bitrate == 0)       ? "slcan — -s8 가정" : "⚠ 1Mbps 아님");
        fclose(f);
    } else {
        printf("  [3] Bitrate: slcan 모드 (파일에서 확인 불가, -s8 = 1Mbps 가정)\n");
    }

    return true;
}

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
    for (int i = 0; i < len; ++i) printf("%02X ", data[i]);
}

// ════════════════════════════════════════════════════════════════
//  RMD: 0xA1 토크 명령 (iq=0 → 움직임 없음)
// ════════════════════════════════════════════════════════════════

static MotorResult testRmdTorque(int motor_id, int16_t iq_cmd = 0)
{
    MotorResult res;

    uint32_t tx_id      = arm_sdk::RMDProtocol::txId(motor_id);
    uint32_t expected_rx = arm_sdk::RMDProtocol::rxId(motor_id);

    uint8_t cmd[8] = {0};
    cmd[0] = RMD_CMD_TORQUE_CTRL;
    cmd[4] = static_cast<uint8_t>(iq_cmd & 0xFF);
    cmd[5] = static_cast<uint8_t>((iq_cmd >> 8) & 0xFF);

    printf("    TX → ID=0x%03X  Cmd=0xA1(Torque iq=%d)  Data: ", tx_id, iq_cmd);
    printHex(cmd, 8); printf("\n");

    if (!g_can.sendFrame(tx_id, cmd, 8, false)) {
        printf("    [FAIL] sendFrame 실패\n");
        return res;
    }

    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;
    auto t0 = std::chrono::steady_clock::now();

    for (int attempt = 0; attempt < 30; ++attempt) {
        bool ext = false;
        if (g_can.receiveFrame(rx_id, rx_data, rx_len, 5, &ext)) {
            if (!ext && rx_id == expected_rx && rx_data[0] == RMD_CMD_TORQUE_CTRL) {
                auto dt = std::chrono::steady_clock::now() - t0;
                double ms = std::chrono::duration<double, std::milli>(dt).count();

                printf("    RX ← ID=0x%03X  Data: ", rx_id);
                printHex(rx_data, rx_len); printf(" (%.1fms)\n", ms);

                int8_t  temp   = static_cast<int8_t>(rx_data[1]);
                int16_t iq_fb  = static_cast<int16_t>(
                                    rx_data[2] | (uint16_t(rx_data[3]) << 8));
                int16_t speed  = static_cast<int16_t>(
                                    rx_data[4] | (uint16_t(rx_data[5]) << 8));
                uint16_t enc   = static_cast<uint16_t>(
                                    rx_data[6] | (uint16_t(rx_data[7]) << 8));

                double cur_a  = iq_fb * 0.01;
                double spd    = static_cast<double>(speed);
                double deg    = enc * 360.0 / 65536.0;

                res.ok      = true;
                res.pos_deg = deg;
                res.vel_dps = spd;
                res.torque  = cur_a * RMD_TORQUE_CONST;
                res.temp_c  = static_cast<double>(temp);

                printf("    ✓ Temp=%d°C  iq=%.2fA(%.3fNm)  Speed=%.0f dps  Enc=%d(%.1f°)\n",
                       (int)temp, cur_a, res.torque, spd, enc, deg);
                return res;
            }
        }
    }
    printf("    [FAIL] 응답 없음 (timeout)\n");
    return res;
}

// ════════════════════════════════════════════════════════════════
//  Robstride: Enable → Feedback → 즉시 Disable (움직임 없음)
// ════════════════════════════════════════════════════════════════

static MotorResult testRobstrideFeedback(int motor_id)
{
    MotorResult res;
    uint32_t arb_id;
    uint8_t data[8];

    arm_sdk::RobstrideProtocol::buildEnable(arb_id, data,
        static_cast<uint8_t>(motor_id));

    printf("    TX → ArbID=0x%08X (Enable)  Data: ", arb_id);
    printHex(data, 8); printf("\n");

    if (!g_can.sendFrame(arb_id, data, 8, true)) {
        printf("    [FAIL] sendFrame 실패\n");
        return res;
    }

    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;
    auto t0 = std::chrono::steady_clock::now();

    for (int attempt = 0; attempt < 50; ++attempt) {
        bool ext = false;
        if (g_can.receiveFrame(rx_id, rx_data, rx_len, 10, &ext)) {
            if (ext) {
                auto dt = std::chrono::steady_clock::now() - t0;
                double ms = std::chrono::duration<double, std::milli>(dt).count();

                printf("    RX ← ArbID=0x%08X  Data: ", rx_id);
                printHex(rx_data, rx_len); printf(" (%.1fms)\n", ms);

                auto st = arm_sdk::RobstrideProtocol::parseFeedback(
                    rx_id, rx_data,
                    arm_sdk::RS03_TORQUE_MIN, arm_sdk::RS03_TORQUE_MAX,
                    arm_sdk::RS03_VEL_MIN,    arm_sdk::RS03_VEL_MAX);

                if (st.valid && st.motor_id == static_cast<uint8_t>(motor_id)) {
                    res.ok      = true;
                    res.pos_deg = st.position_rad * 180.0 / M_PI;
                    res.vel_dps = st.velocity_rads * 180.0 / M_PI;
                    res.torque  = st.torque_nm;
                    res.temp_c  = st.temperature;
                    res.error   = st.error_bits;

                    printf("    ✓ Mode=%s  Pos=%.2f°  Vel=%.2f°/s  Torque=%.3fNm  Temp=%.1f°C",
                           arm_sdk::RobstrideProtocol::modeString(st.mode),
                           res.pos_deg, res.vel_dps, res.torque, res.temp_c);
                    if (st.error_bits) printf("  ⚠ err=0x%02X", st.error_bits);
                    printf("\n");
                    break;
                }
            }
        }
    }

    // 즉시 Disable — 안전
    arm_sdk::RobstrideProtocol::buildDisable(arb_id, data,
        static_cast<uint8_t>(motor_id));
    g_can.sendFrame(arb_id, data, 8, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    if (!res.ok) printf("    [FAIL] Feedback 응답 없음\n");
    return res;
}

// ════════════════════════════════════════════════════════════════
//  결과 테이블
// ════════════════════════════════════════════════════════════════

struct MotorEntry {
    const char* label;
    const char* type;
    int id;
    MotorResult result;
};

static void printTable(MotorEntry* m, int n)
{
    printf("┌──────────────┬──────────┬─────┬──────────┬──────────┬──────────┬────────┬──────────────┬────────┐\n");
    printf("│    Joint     │   Type   │  ID │ Pos(deg) │ Vel(d/s) │ Torq(Nm) │ Temp°C │  TX Command  │ Status │\n");
    printf("├──────────────┼──────────┼─────┼──────────┼──────────┼──────────┼────────┼──────────────┼────────┤\n");
    for (int i = 0; i < n; ++i) {
        if (m[i].result.ok)
            printf("│ %-12s │ %-8s │ %3d │ %8.2f │ %8.2f │ %8.3f │ %5.1f  │ %-12s │   ✓    │\n",
                   m[i].label, m[i].type, m[i].id,
                   m[i].result.pos_deg, m[i].result.vel_dps,
                   m[i].result.torque, m[i].result.temp_c,
                   m[i].result.cmd_type);
        else
            printf("│ %-12s │ %-8s │ %3d │   ---    │   ---    │   ---    │  ---   │ %-12s │   ✗    │\n",
                   m[i].label, m[i].type, m[i].id, m[i].result.cmd_type);
    }
    printf("└──────────────┴──────────┴─────┴──────────┴──────────┴──────────┴────────┴──────────────┴────────┘\n");
}

// ════════════════════════════════════════════════════════════════
//  Main
// ════════════════════════════════════════════════════════════════

int main(int argc, char* argv[])
{
    signal(SIGINT, signalHandler);
    std::string can_if = "can0";
    bool can_if_set = false;
    bool period_set = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--comm-only") {
            g_comm_only = true;
            continue;
        }
        if (!can_if_set) {
            can_if = arg;
            can_if_set = true;
            continue;
        }
        if (!period_set) {
            g_loop_period_ms = std::atoi(arg.c_str());
            if (g_loop_period_ms < 2) g_loop_period_ms = 2;
            if (g_loop_period_ms > 1000) g_loop_period_ms = 1000;
            period_set = true;
            continue;
        }

        printf("Usage: %s [can_if] [period_ms] [--comm-only]\n", argv[0]);
        return 1;
    }

    printf("╔═══════════════════════════════════════════════════════════════╗\n");
    printf("║  6DOF Motor Comm / Hold Test                                 ║\n");
    printf("╠═══════════════════════════════════════════════════════════════╣\n");
    printf("║  J1: Robstride03   ID=%-3d   (Extended CAN 29-bit)          ║\n", RS_MOTOR_1_ID);
    printf("║  J2: Robstride03   ID=%-3d   (Extended CAN 29-bit)          ║\n", RS_MOTOR_2_ID);
    printf("║  J3: MyActuator    ID=%-3d   (Standard CAN 11-bit)          ║\n", RMD_MOTOR_1_ID);
    printf("║  J4: MyActuator    ID=%-3d   (Standard CAN 11-bit)          ║\n", RMD_MOTOR_2_ID);
    printf("║  J5: MyActuator    ID=%-3d   (Standard CAN 11-bit)          ║\n", RMD_MOTOR_3_ID);
    printf("║  J6: MyActuator    ID=%-3d   (Standard CAN 11-bit)          ║\n", RMD_MOTOR_4_ID);
    printf("║                                                               ║\n");
    printf("║  ✓ CAN 프레임 타입이 달라 ID 겹침(RS=1, RMD=1) 무관        ║\n");
    printf("║  Loop: %dms (%dHz)  — 변경: ./test_read_only can0 <ms>     ║\n",
           g_loop_period_ms, 1000 / g_loop_period_ms);
    if (g_comm_only) {
        printf("║  ✓ --comm-only: Step 4까지 송수신 확인 후 즉시 종료         ║\n");
    } else {
        printf("║  ✓ Step 5에서 전체 위치 홀드 (움직임 없음)                  ║\n");
    }
    printf("╚═══════════════════════════════════════════════════════════════╝\n\n");

    // ══════════════════════════════════════════════════════════
    // Step 1: SocketCAN 인터페이스 확인
    // ══════════════════════════════════════════════════════════
    printf("═══ Step 1: SocketCAN 인터페이스 확인 (%s) ══════════\n\n", can_if.c_str());
    if (!checkSocketCAN(can_if)) {
        printf("\n  ✗ SocketCAN 준비 안됨:\n");
        printf("    sudo slcand -o -c -s8 /dev/ttyACM0 %s\n", can_if.c_str());
        printf("    sudo ip link set up %s\n\n", can_if.c_str());
        return 1;
    }
    printf("\n");

    // ══════════════════════════════════════════════════════════
    // Step 2: 소켓 열기
    // ══════════════════════════════════════════════════════════
    printf("═══ Step 2: 소켓 열기 ═══════════════════════════════\n");
    if (!g_can.open(can_if)) {
        printf("  [FAIL] %s 소켓 열기 실패\n", can_if.c_str());
        return 1;
    }
    printf("  [OK] %s 소켓 열림\n\n", can_if.c_str());
    flushRx(5);

    // ══════════════════════════════════════════════════════════
    // Step 3: 각 모터 개별 테스트 (하드코딩)
    // ══════════════════════════════════════════════════════════
    printf("═══ Step 3: 개별 모터 토크 테스트 (iq=0) ════════════\n");
    printf("  ⚠ 토크 0 → 모터가 움직이지 않습니다\n\n");

    MotorEntry motors[6] = {
        {"J1 Shoulder",  "RS03",  RS_MOTOR_1_ID,  {}},
        {"J2 UpperArm",  "RS03",  RS_MOTOR_2_ID,  {}},
        {"J3 Elbow",     "X4-36", RMD_MOTOR_1_ID, {}},
        {"J4 Wrist1",    "X4-36", RMD_MOTOR_2_ID, {}},
        {"J5 Wrist2",    "X4-36", RMD_MOTOR_3_ID, {}},
        {"J6 Wrist3",    "X4-36", RMD_MOTOR_4_ID, {}},
    };

    int pass = 0, fail = 0;

    // ── J1: Robstride03 ID=1 ──────────────────────────────
    printf("  ── [J1] Robstride03 ID=%d (Shoulder) ──\n", RS_MOTOR_1_ID);
    motors[0].result = testRobstrideFeedback(RS_MOTOR_1_ID);
    (motors[0].result.ok) ? pass++ : fail++;
    printf("\n"); flushRx();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    if (!g_running) { g_can.close(); return 0; }

    // ── J2: Robstride03 ID=127 ────────────────────────────
    printf("  ── [J2] Robstride03 ID=%d (Upper Arm) ──\n", RS_MOTOR_2_ID);
    motors[1].result = testRobstrideFeedback(RS_MOTOR_2_ID);
    (motors[1].result.ok) ? pass++ : fail++;
    printf("\n"); flushRx();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    if (!g_running) { g_can.close(); return 0; }

    // ── J3: MyActuator ID=1 ───────────────────────────────
    printf("  ── [J3] MyActuator X4-36 ID=%d (Elbow) ──\n", RMD_MOTOR_1_ID);
    motors[2].result = testRmdTorque(RMD_MOTOR_1_ID, 0);
    (motors[2].result.ok) ? pass++ : fail++;
    printf("\n"); flushRx();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    if (!g_running) { g_can.close(); return 0; }

    // ── J4: MyActuator ID=2 ───────────────────────────────
    printf("  ── [J4] MyActuator X4-36 ID=%d (Wrist 1) ──\n", RMD_MOTOR_2_ID);
    motors[3].result = testRmdTorque(RMD_MOTOR_2_ID, 0);
    (motors[3].result.ok) ? pass++ : fail++;
    printf("\n"); flushRx();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    if (!g_running) { g_can.close(); return 0; }

    // ── J5: MyActuator ID=3 ───────────────────────────────
    printf("  ── [J5] MyActuator X4-36 ID=%d (Wrist 2) ──\n", RMD_MOTOR_3_ID);
    motors[4].result = testRmdTorque(RMD_MOTOR_3_ID, 0);
    (motors[4].result.ok) ? pass++ : fail++;
    printf("\n"); flushRx();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    if (!g_running) { g_can.close(); return 0; }

    // ── J6: MyActuator ID=4 ───────────────────────────────
    printf("  ── [J6] MyActuator X4-36 ID=%d (Wrist 3) ──\n", RMD_MOTOR_4_ID);
    motors[5].result = testRmdTorque(RMD_MOTOR_4_ID, 0);
    (motors[5].result.ok) ? pass++ : fail++;
    printf("\n"); flushRx();

    if (!g_running) { g_can.close(); return 0; }

    // ══════════════════════════════════════════════════════════
    // Step 4: 결과 요약
    // ══════════════════════════════════════════════════════════
    printf("═══ Step 4: 결과 요약 ═══════════════════════════════\n\n");
    printTable(motors, 6);
    printf("\n  결과: %d PASS / %d FAIL / 6 전체\n\n", pass, fail);

    if (fail > 0) {
        printf("  ⚠ 응답 없는 모터 확인:\n");
        printf("    1. 모터 전원 ON?\n");
        printf("    2. CAN 배선 (CAN_H, CAN_L, GND) + 120Ω 종단저항\n");
        printf("    3. 모터 ID: RS03 → %d, %d  /  RMD → %d, %d, %d, %d\n",
               RS_MOTOR_1_ID, RS_MOTOR_2_ID,
               RMD_MOTOR_1_ID, RMD_MOTOR_2_ID, RMD_MOTOR_3_ID, RMD_MOTOR_4_ID);
        printf("    4. slcand -s8 = 1Mbps 확인\n\n");
    }

    if (g_comm_only) {
        printf("  [OK] --comm-only: 위치 홀드 없이 통신 확인만 수행했습니다.\n");
        g_can.close();
        return (fail > 0) ? 1 : 0;
    }

    if (!g_running || pass == 0) {
        g_can.close();
        return (fail > 0) ? 1 : 0;
    }

    // ══════════════════════════════════════════════════════════
    // Step 5: 연속 모니터링 (토크 0 반복, Ctrl+C 종료)
    // ══════════════════════════════════════════════════════════
    printf("═══ Step 5: 위치 유지 모니터링 (Ctrl+C 종료) ═══════\n");
    printf("  ⚠ 현재 위치를 유지합니다 (Position Hold)\n\n");

    // ── 현재 위치 저장 (Hold 목표) ────────────────────────
    float rs_hold_pos[2]  = {0};
    float rmd_hold_pos[4] = {0};

    // RS: 초기 위치 = 개별 테스트에서 읽은 값 (deg→rad) — RS는 절대 위치이므로 OK
    for (int i = 0; i < 2; ++i) {
        if (motors[i].result.ok)
            rs_hold_pos[i] = static_cast<float>(motors[i].result.pos_deg * M_PI / 180.0);
    }

    // RMD: 0x9C Read Status 2로 멀티턴 위치를 정확히 읽기
    //       (0xA1 응답의 encoder는 싱글턴이므로 위치 제어에 사용 불가)
    printf("  [RMD] 0x9C로 멀티턴 위치 읽기...\n");
    for (int i = 0; i < 4; ++i) {
        int rmd_id = (i == 0) ? RMD_MOTOR_1_ID : (i == 1) ? RMD_MOTOR_2_ID :
                     (i == 2) ? RMD_MOTOR_3_ID : RMD_MOTOR_4_ID;
        uint8_t cmd[8] = {0};
        arm_sdk::RMDProtocol::buildReadStatus2(cmd);
        uint32_t tx_id = arm_sdk::RMDProtocol::txId(rmd_id);
        uint32_t rx_ex = arm_sdk::RMDProtocol::rxId(rmd_id);

        flushRx(1);
        if (g_can.sendFrame(tx_id, cmd, 8, false)) {
            uint32_t rx_id; uint8_t rx[8], rl;
            for (int a = 0; a < 20; ++a) {
                bool ext = false;
                if (g_can.receiveFrame(rx_id, rx, rl, 5, &ext)) {
                    if (!ext && rx_id == rx_ex && rx[0] == 0x9C) {
                        auto st = arm_sdk::RMDProtocol::parseStatus2(rx, rl, RMD_TORQUE_CONST);
                        if (st.valid) {
                            rmd_hold_pos[i] = static_cast<float>(st.position_rad);
                            printf("    RMD ID=%d → 멀티턴 %.2f° (%.4f rad)\n",
                                   rmd_id,
                                   st.position_rad * 180.0 / M_PI,
                                   st.position_rad);
                            break;
                        }
                    }
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // ── RMD: 가속도 설정 (0x43) — 0xA4 위치 제어에 필요 ──
    printf("  [RMD] 가속도 설정 (0x43)...\n");
    for (int rmd_id : {RMD_MOTOR_1_ID, RMD_MOTOR_2_ID, RMD_MOTOR_3_ID, RMD_MOTOR_4_ID}) {
        uint8_t cmd[8] = {0};
        arm_sdk::RMDProtocol::buildSetAcceleration(cmd, 500);  // 500 dps²
        g_can.sendFrame(arm_sdk::RMDProtocol::txId(rmd_id), cmd, 8, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        flushRx(3);
        printf("    RMD ID=%d → accel=500 dps²\n", rmd_id);
    }

    // ── Robstride: Enable + Position Mode 진입 ────────────
    for (int i = 0; i < 2; ++i) {
        int rs_id = (i == 0) ? RS_MOTOR_1_ID : RS_MOTOR_2_ID;
        uint32_t arb_id; uint8_t data[8];

        // Enable
        arm_sdk::RobstrideProtocol::buildEnable(arb_id, data,
            static_cast<uint8_t>(rs_id));
        g_can.sendFrame(arb_id, data, 8, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        flushRx(3);

        // Position Mode (run_mode = 1)
        arm_sdk::RobstrideProtocol::buildWriteParam(arb_id, data,
            static_cast<uint8_t>(rs_id), arm_sdk::RS_PARAM_RUN_MODE, 1.0f);
        g_can.sendFrame(arb_id, data, 8, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        flushRx(3);

        // Speed limit
        arm_sdk::RobstrideProtocol::buildWriteParam(arb_id, data,
            static_cast<uint8_t>(rs_id), arm_sdk::RS_PARAM_LIMIT_SPD, 5.0f);
        g_can.sendFrame(arb_id, data, 8, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        flushRx(3);
    }
    printf("  [OK] Robstride Position Mode 진입 완료\n");

    int cycle = 0;
    while (g_running) {
        auto t0 = std::chrono::steady_clock::now();
        double dt_rs_ms[2] = {0}, dt_rmd_ms[4] = {0};

        // ══ TX burst: 6개 위치 명령을 연속 전송 ═══════════
        // RS: LOC_REF 위치 명령 (피드백도 받음)
        for (int i = 0; i < 2; ++i) {
            int rs_id = (i == 0) ? RS_MOTOR_1_ID : RS_MOTOR_2_ID;
            uint32_t arb_id; uint8_t data[8];
            arm_sdk::RobstrideProtocol::buildWriteParam(
                arb_id, data, static_cast<uint8_t>(rs_id),
                arm_sdk::RS_PARAM_LOC_REF, rs_hold_pos[i]);
            g_can.sendFrame(arb_id, data, 8, true);
        }
        // RMD: 0xA4 Position Ctrl 2 (현재 위치 유지)
        int rmd_ids[4] = {RMD_MOTOR_1_ID, RMD_MOTOR_2_ID, RMD_MOTOR_3_ID, RMD_MOTOR_4_ID};
        uint32_t rmd_expected[4];
        for (int i = 0; i < 4; ++i) {
            uint8_t cmd[8] = {0};
            arm_sdk::MotorCommand mc;
            mc.position_rad    = static_cast<double>(rmd_hold_pos[i]);
            mc.velocity_rads   = 0.0;
            mc.default_vel_dps = 50.0;
            mc.max_vel_dps     = 720.0;
            arm_sdk::RMDProtocol::buildPositionCtrl2(cmd, mc);
            rmd_expected[i] = arm_sdk::RMDProtocol::rxId(rmd_ids[i]);
            g_can.sendFrame(arm_sdk::RMDProtocol::txId(rmd_ids[i]), cmd, 8, false);
        }

        // ══ RX collect: deadline 방식 — TX 후 최대 5ms만 대기 ═══
        auto t_rx = std::chrono::steady_clock::now();
        auto deadline = t_rx + std::chrono::milliseconds(5);
        bool rs_done[2] = {false}, rmd_done[4] = {false};
        int total_got = 0;

        while (total_got < 6 && g_running &&
               std::chrono::steady_clock::now() < deadline) {
            uint32_t rx_id; uint8_t rx_data[8], rx_len;
            bool ext = false;
            if (!g_can.receiveFrame(rx_id, rx_data, rx_len, 1, &ext))
                continue;

            if (ext) {
                // Robstride 응답
                auto st = arm_sdk::RobstrideProtocol::parseFeedback(
                    rx_id, rx_data,
                    arm_sdk::RS03_TORQUE_MIN, arm_sdk::RS03_TORQUE_MAX,
                    arm_sdk::RS03_VEL_MIN,    arm_sdk::RS03_VEL_MAX);
                if (!st.valid) continue;
                for (int i = 0; i < 2; ++i) {
                    int rs_id = (i == 0) ? RS_MOTOR_1_ID : RS_MOTOR_2_ID;
                    if (!rs_done[i] && st.motor_id == static_cast<uint8_t>(rs_id)) {
                        motors[i].result.ok      = true;
                        motors[i].result.pos_deg = st.position_rad * 180.0 / M_PI;
                        motors[i].result.vel_dps = st.velocity_rads * 180.0 / M_PI;
                        motors[i].result.torque  = st.torque_nm;
                        motors[i].result.temp_c  = st.temperature;
                        motors[i].result.error   = st.error_bits;
                        motors[i].result.cmd_type = "PosHold";
                        rs_done[i] = true;
                        dt_rs_ms[i] = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - t_rx).count();
                        total_got++;
                        break;
                    }
                }
            } else {
                // RMD 응답
                if (rx_data[0] != arm_sdk::RMD_CMD_POSITION_CTRL2 &&
                    rx_data[0] != RMD_CMD_TORQUE_CTRL) continue;
                for (int i = 0; i < 4; ++i) {
                    if (!rmd_done[i] && rx_id == rmd_expected[i]) {
                        int8_t  temp  = static_cast<int8_t>(rx_data[1]);
                        int16_t iq_fb = static_cast<int16_t>(
                            rx_data[2] | (uint16_t(rx_data[3]) << 8));
                        int16_t speed = static_cast<int16_t>(
                            rx_data[4] | (uint16_t(rx_data[5]) << 8));
                        uint16_t enc  = static_cast<uint16_t>(
                            rx_data[6] | (uint16_t(rx_data[7]) << 8));

                        int idx = 2 + i;
                        motors[idx].result.ok      = true;
                        motors[idx].result.pos_deg = enc * 360.0 / 65536.0;
                        motors[idx].result.vel_dps = double(speed);
                        motors[idx].result.torque  = iq_fb * 0.01 * RMD_TORQUE_CONST;
                        motors[idx].result.temp_c  = double(temp);
                        motors[idx].result.cmd_type = "0xA4 Pos";
                        rmd_done[i] = true;
                        dt_rmd_ms[i] = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - t_rx).count();
                        total_got++;
                        break;
                    }
                }
            }
        }

        cycle++;

        auto el = std::chrono::steady_clock::now() - t0;
        double proc_ms = std::chrono::duration<double, std::milli>(el).count();

        auto remain = std::chrono::milliseconds(g_loop_period_ms) - el;
        if (remain > std::chrono::milliseconds(0))
            std::this_thread::sleep_for(remain);

        auto total_el = std::chrono::steady_clock::now() - t0;
        double total_ms = std::chrono::duration<double, std::milli>(total_el).count();
        double actual_hz = (total_ms > 0) ? 1000.0 / total_ms : 0;

        // 화면 갱신 (매 사이클)
        printf("\033[2J\033[H");
        printf("═══ 연속 모니터링 [Cycle %d]  %.1fms(처리%.1f)  %.0fHz(목표%dHz)  %d/6응답  Ctrl+C ═══\n",
               cycle, total_ms, proc_ms, actual_hz, 1000 / g_loop_period_ms, total_got);
        printf("  ⚠ Position Hold — 현재 위치 유지 중  │  ./test_read_only can0 <ms>\n");
        printf("  ┌─ RX소요(ms): RS1=%.1f RS2=%.1f │ RMD: ",
               dt_rs_ms[0], dt_rs_ms[1]);
        for (int i = 0; i < 4; ++i) printf("%d=%.1f ", rmd_ids[i], dt_rmd_ms[i]);
        printf("─┐\n\n");
        printTable(motors, 6);
    }

    // ══════════════════════════════════════════════════════════
    // 종료
    // ══════════════════════════════════════════════════════════
    printf("\n\n═══ 종료 ════════════════════════════════════════════\n");

    for (int rs_id : {RS_MOTOR_1_ID, RS_MOTOR_2_ID}) {
        uint32_t arb_id; uint8_t data[8];
        arm_sdk::RobstrideProtocol::buildDisable(arb_id, data,
            static_cast<uint8_t>(rs_id));
        g_can.sendFrame(arb_id, data, 8, true);
    }
    // RMD: 토크 0으로 해제
    for (int rmd_id : {RMD_MOTOR_1_ID, RMD_MOTOR_2_ID, RMD_MOTOR_3_ID, RMD_MOTOR_4_ID}) {
        uint8_t cmd[8] = {0};
        cmd[0] = RMD_CMD_TORQUE_CTRL;  // 0xA1 iq=0 → 토크 해제
        g_can.sendFrame(arm_sdk::RMDProtocol::txId(rmd_id), cmd, 8, false);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    g_can.close();
    printf("[DONE] 안전 종료 완료\n");
    return 0;
}
