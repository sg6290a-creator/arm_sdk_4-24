/**
 * ============================================================================
 * test_socketcan — SocketCAN(can0)으로 Robstride 모터 통신 + 위치 제어 테스트
 * ============================================================================
 *
 * CANable2 등 SocketCAN 어댑터용.
 *   - Robstride: Enable → 통신 확인 → 위치 제어 (+30°→원위치) → Disable
 *
 * 사전 준비:
 *   sudo slcand -o -c -s8 /dev/ttyACM0 can0
 *   sudo ip link set up can0
 *
 * 사용법:
 *   sudo ./test_socketcan [motor_id] [can_if] [move_deg]
 *   sudo ./test_socketcan                ← motor ID=1, can0, +30°
 *   sudo ./test_socketcan 1 can0         ← ID=1, can0, +30°
 *   sudo ./test_socketcan 1 can0 45      ← ID=1, can0, +45°
 *   sudo ./test_socketcan 1 can0 0       ← 움직임 없이 통신만 확인
 *
 * 빌드:
 *   cmake .. -DARM_SDK_BUILD_TESTS=ON && make test_socketcan
 *
 * ============================================================================
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <chrono>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static volatile bool g_running = true;
void signalHandler(int) { g_running = false; }

// ════════════════════════════════════════════════════════════════
//  Robstride CAN protocol constants (Extended CAN, 29-bit)
// ════════════════════════════════════════════════════════════════

constexpr uint8_t RS_MSG_ENABLE      = 0x03;
constexpr uint8_t RS_MSG_DISABLE     = 0x04;
constexpr uint8_t RS_MSG_FEEDBACK    = 0x02;
constexpr uint8_t RS_MSG_READ_PARAM  = 0x11;
constexpr uint8_t RS_MSG_WRITE_PARAM = 0x12;

constexpr uint16_t RS_PARAM_MECH_POS = 0x7019;
constexpr uint16_t RS_PARAM_MECH_VEL = 0x701B;
constexpr uint16_t RS_PARAM_VBUS     = 0x701C;

constexpr uint8_t HOST_CAN_ID = 0x00;

constexpr float P_MIN = -12.5f;
constexpr float P_MAX =  12.5f;
constexpr float V_MIN = -44.0f;
constexpr float V_MAX =  44.0f;
constexpr float T_MIN = -17.0f;
constexpr float T_MAX =  17.0f;

/// Build extended CAN arbitration ID: [msg_type:5][data1:16][motor_id:8]
static uint32_t makeArbId(uint8_t msg_type, uint16_t data1, uint8_t motor_id)
{
    return (static_cast<uint32_t>(msg_type) << 24) |
           (static_cast<uint32_t>(data1) << 8) |
           motor_id;
}

static float uint16ToFloat(uint16_t x, float x_min, float x_max)
{
    return (x_max - x_min) * static_cast<float>(x) / 65535.0f + x_min;
}

static void printHex(const uint8_t* data, int len)
{
    for (int i = 0; i < len; ++i) printf("%02X ", data[i]);
}

// ════════════════════════════════════════════════════════════════
//  SocketCAN helpers
// ════════════════════════════════════════════════════════════════

static int openSocketCAN(const char* ifname)
{
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket");
        return -1;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl SIOCGIFINDEX");
        close(s);
        return -1;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(s);
        return -1;
    }

    // Set receive timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500000;  // 500ms
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    return s;
}

static bool sendExtFrame(int sock, uint32_t id, const uint8_t* data, uint8_t len)
{
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = id | CAN_EFF_FLAG;  // Extended frame
    frame.can_dlc = len;
    std::memcpy(frame.data, data, len);

    int ret = write(sock, &frame, sizeof(frame));
    return (ret == sizeof(frame));
}

static bool recvFrame(int sock, uint32_t* id, uint8_t* data, uint8_t* len, int timeout_ms = 500)
{
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct can_frame frame;
    int ret = read(sock, &frame, sizeof(frame));
    if (ret < 0) return false;

    if (id) *id = frame.can_id & CAN_EFF_MASK;
    if (data) std::memcpy(data, frame.data, frame.can_dlc);
    if (len) *len = frame.can_dlc;
    return true;
}

/// Flush pending frames
static void flushRx(int sock)
{
    uint32_t id; uint8_t d[8], l;
    while (recvFrame(sock, &id, d, &l, 1)) {}
}

// ════════════════════════════════════════════════════════════════
//  Robstride communication tests
// ════════════════════════════════════════════════════════════════

static bool readParam(int sock, uint8_t motor_id, uint16_t param_id,
                      float& value, const char* name)
{
    uint8_t tx_data[8] = {0};
    std::memcpy(&tx_data[0], &param_id, 2);

    uint32_t tx_id = makeArbId(RS_MSG_READ_PARAM, HOST_CAN_ID, motor_id);

    printf("    TX → ID=0x%08X  Data: ", tx_id);
    printHex(tx_data, 8);
    printf("  (%s)\n", name);

    if (!sendExtFrame(sock, tx_id, tx_data, 8)) {
        printf("    [FAIL] sendFrame failed\n");
        return false;
    }

    uint32_t rx_id; uint8_t rx_data[8], rx_len;
    auto t0 = std::chrono::steady_clock::now();

    for (int attempt = 0; attempt < 30; ++attempt) {
        if (recvFrame(sock, &rx_id, rx_data, &rx_len, 10)) {
            auto dt = std::chrono::steady_clock::now() - t0;
            double ms = std::chrono::duration<double, std::milli>(dt).count();

            uint8_t rx_msg_type = (rx_id >> 24) & 0x1F;

            printf("    RX ← ID=0x%08X  Data: ", rx_id);
            printHex(rx_data, rx_len);
            printf(" (%.1fms)\n", ms);

            if (rx_msg_type == RS_MSG_READ_PARAM) {
                std::memcpy(&value, &rx_data[4], 4);
                printf("    ✓ %s = %.4f\n", name, value);
                return true;
            }
        }
    }
    printf("    [FAIL] No response for %s\n", name);
    return false;
}

static bool testRobstrideEnable(int sock, uint8_t motor_id)
{
    printf("\n  ── Robstride Motor (ID=%d) Enable Test ──\n", motor_id);

    flushRx(sock);

    // Enable
    uint8_t tx_data[8] = {0};
    uint32_t tx_id = makeArbId(RS_MSG_ENABLE, HOST_CAN_ID, motor_id);

    printf("    TX → ID=0x%08X  Data: ", tx_id);
    printHex(tx_data, 8);
    printf("  (Enable)\n");

    if (!sendExtFrame(sock, tx_id, tx_data, 8)) {
        printf("    [FAIL] sendFrame failed\n");
        return false;
    }

    // Wait for feedback
    uint32_t rx_id; uint8_t rx_data[8], rx_len;
    auto t0 = std::chrono::steady_clock::now();
    bool got_feedback = false;

    for (int attempt = 0; attempt < 50; ++attempt) {
        if (recvFrame(sock, &rx_id, rx_data, &rx_len, 10)) {
            auto dt = std::chrono::steady_clock::now() - t0;
            double ms = std::chrono::duration<double, std::milli>(dt).count();

            uint8_t rx_msg_type = (rx_id >> 24) & 0x1F;

            printf("    RX ← ID=0x%08X  Data: ", rx_id);
            printHex(rx_data, rx_len);
            printf(" (%.1fms)\n", ms);

            if (rx_msg_type == RS_MSG_FEEDBACK) {
                // Parse feedback
                uint16_t pos_raw = (static_cast<uint16_t>(rx_data[1]) << 8) | rx_data[0];
                uint16_t vel_raw = (static_cast<uint16_t>(rx_data[3]) << 8) | rx_data[2];
                uint16_t torq_raw = (static_cast<uint16_t>(rx_data[5]) << 8) | rx_data[4];

                float pos = uint16ToFloat(pos_raw, P_MIN, P_MAX);
                float vel = uint16ToFloat(vel_raw, V_MIN, V_MAX);
                float torq = uint16ToFloat(torq_raw, T_MIN, T_MAX);

                uint8_t mode = rx_data[6] >> 5;
                uint8_t errors = rx_data[6] & 0x1F;

                printf("    ✓ Feedback: pos=%.3f rad (%.1f°), vel=%.2f rad/s, torque=%.2f Nm\n",
                       pos, pos * 180.0 / M_PI, vel, torq);
                printf("    ✓ Mode=%d, Errors=0x%02X\n", mode, errors);

                got_feedback = true;
                break;
            }
        }
    }

    if (!got_feedback) {
        printf("    [FAIL] No feedback received\n");
        return false;
    }

    return true;
}

// ════════════════════════════════════════════════════════════════
//  Robstride write parameter
// ════════════════════════════════════════════════════════════════

static bool writeParam(int sock, uint8_t motor_id, uint16_t param_id,
                       float value, const char* name)
{
    uint8_t tx_data[8] = {0};
    std::memcpy(&tx_data[0], &param_id, 2);  // bytes 0-1: param index
    // bytes 4-7: float value
    std::memcpy(&tx_data[4], &value, 4);

    uint32_t tx_id = makeArbId(RS_MSG_WRITE_PARAM, HOST_CAN_ID, motor_id);

    printf("    TX → ID=0x%08X  Data: ", tx_id);
    printHex(tx_data, 8);
    printf("  (write %s = %.4f)\n", name, value);

    if (!sendExtFrame(sock, tx_id, tx_data, 8)) {
        printf("    [FAIL] sendFrame failed\n");
        return false;
    }

    // Drain response
    uint32_t rx_id; uint8_t rx_data[8], rx_len;
    for (int i = 0; i < 10; ++i) {
        if (recvFrame(sock, &rx_id, rx_data, &rx_len, 10)) {
            printf("    RX ← ID=0x%08X  Data: ", rx_id);
            printHex(rx_data, rx_len);
            printf("\n");
        }
    }
    return true;
}

static bool writeParamU8(int sock, uint8_t motor_id, uint16_t param_id,
                         uint8_t value, const char* name)
{
    uint8_t tx_data[8] = {0};
    std::memcpy(&tx_data[0], &param_id, 2);
    tx_data[4] = value;

    uint32_t tx_id = makeArbId(RS_MSG_WRITE_PARAM, HOST_CAN_ID, motor_id);

    printf("    TX → ID=0x%08X  Data: ", tx_id);
    printHex(tx_data, 8);
    printf("  (write %s = %d)\n", name, value);

    if (!sendExtFrame(sock, tx_id, tx_data, 8)) {
        printf("    [FAIL] sendFrame failed\n");
        return false;
    }

    uint32_t rx_id; uint8_t rx_data[8], rx_len;
    for (int i = 0; i < 10; ++i) {
        recvFrame(sock, &rx_id, rx_data, &rx_len, 10);
    }
    return true;
}

// ════════════════════════════════════════════════════════════════
//  Position mode setup & control
// ════════════════════════════════════════════════════════════════

constexpr uint16_t RS_PARAM_RUN_MODE     = 0x7005;
constexpr uint16_t RS_PARAM_LOC_REF      = 0x7016;
constexpr uint16_t RS_PARAM_LIMIT_SPD    = 0x7017;
constexpr uint16_t RS_PARAM_LIMIT_CUR    = 0x7018;
constexpr uint8_t  RS_MODE_POSITION      = 1;

static bool setupPositionMode(int sock, uint8_t motor_id, float speed_limit = 5.0f)
{
    printf("\n  ── Setup Position Mode (ID=%d) ──\n", motor_id);

    // 1) Disable first (mode change requires disable)
    uint8_t dis_data[8] = {0};
    uint32_t dis_id = makeArbId(RS_MSG_DISABLE, HOST_CAN_ID, motor_id);
    sendExtFrame(sock, dis_id, dis_data, 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    flushRx(sock);

    // 2) Set run_mode = 1 (position)
    writeParamU8(sock, motor_id, RS_PARAM_RUN_MODE, RS_MODE_POSITION, "run_mode");
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // 3) Re-enable
    uint8_t en_data[8] = {0};
    uint32_t en_id = makeArbId(RS_MSG_ENABLE, HOST_CAN_ID, motor_id);
    sendExtFrame(sock, en_id, en_data, 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    flushRx(sock);

    // 4) Set speed limit
    writeParam(sock, motor_id, RS_PARAM_LIMIT_SPD, speed_limit, "limit_spd");

    printf("    ✓ Position mode set (speed_limit=%.1f rad/s)\n", speed_limit);
    return true;
}

static bool moveToPosition(int sock, uint8_t motor_id, float target_rad,
                           float tolerance_deg = 3.0, int timeout_ms = 5000)
{
    printf("\n  ── Move to %.1f° (ID=%d) ──\n", target_rad * 180.0 / M_PI, motor_id);

    // Write position reference
    writeParam(sock, motor_id, RS_PARAM_LOC_REF, target_rad, "loc_ref");

    // Wait for settle
    float tol_rad = tolerance_deg * M_PI / 180.0;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    while (g_running && std::chrono::steady_clock::now() < deadline) {
        // Read current position
        float cur_pos = 0.0f;
        flushRx(sock);
        if (readParam(sock, motor_id, RS_PARAM_MECH_POS, cur_pos, "mech_pos")) {
            float error = std::abs(cur_pos - target_rad);
            printf("    → pos=%.2f° target=%.2f° err=%.2f°\n",
                   cur_pos * 180.0 / M_PI, target_rad * 180.0 / M_PI,
                   error * 180.0 / M_PI);
            if (error < tol_rad) {
                printf("    ✓ Reached target!\n");
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("    [WARN] Timeout — may not have reached target\n");
    return false;
}

// ════════════════════════════════════════════════════════════════

static void testRobstrideDisable(int sock, uint8_t motor_id)
{
    printf("\n  ── Disable Robstride (ID=%d) ──\n", motor_id);

    uint8_t tx_data[8] = {0};
    uint32_t tx_id = makeArbId(RS_MSG_DISABLE, HOST_CAN_ID, motor_id);

    printf("    TX → ID=0x%08X  (Disable)\n", tx_id);
    sendExtFrame(sock, tx_id, tx_data, 8);

    // Drain any response
    uint32_t rx_id; uint8_t rx_data[8], rx_len;
    for (int i = 0; i < 10; ++i) {
        if (recvFrame(sock, &rx_id, rx_data, &rx_len, 10)) {
            printf("    RX ← ID=0x%08X  Data: ", rx_id);
            printHex(rx_data, rx_len);
            printf("\n");
        }
    }

    printf("    ✓ Disabled\n");
}

// ════════════════════════════════════════════════════════════════
//  Main
// ════════════════════════════════════════════════════════════════

int main(int argc, char* argv[])
{
    signal(SIGINT, signalHandler);

    int motor_id    = (argc >= 2) ? std::atoi(argv[1]) : 1;
    const char* can_if = (argc >= 3) ? argv[2] : "can0";
    double move_deg  = (argc >= 4) ? std::atof(argv[3]) : 30.0;

    printf("╔═══════════════════════════════════════════════════╗\n");
    printf("║  SocketCAN Robstride Test (Comm + Move)           ║\n");
    printf("╠═══════════════════════════════════════════════════╣\n");
    printf("║  Motor ID:      %-4d                              ║\n", motor_id);
    printf("║  CAN Interface: %-8s                          ║\n", can_if);
    printf("║  Move:          %.1f°                             ║\n", move_deg);
    printf("╚═══════════════════════════════════════════════════╝\n\n");

    // ── Step 1: Open SocketCAN ─────────────────────────────
    printf("[1] Opening SocketCAN (%s)...\n", can_if);
    int sock = openSocketCAN(can_if);
    if (sock < 0) {
        printf("  [FAIL] Cannot open %s\n", can_if);
        printf("  Run first:\n");
        printf("    sudo slcand -o -c -s8 /dev/ttyACM0 can0\n");
        printf("    sudo ip link set up can0\n");
        return 1;
    }
    printf("  [OK] %s opened (fd=%d)\n\n", can_if, sock);

    // ── Step 2: Enable + Feedback ──────────────────────────
    printf("[2] Testing Enable + Feedback...\n");
    bool ok = testRobstrideEnable(sock, motor_id);

    if (!ok) {
        printf("\n  [FAIL] Motor not responding. Check:\n");
        printf("    1. Motor power ON?\n");
        printf("    2. CAN wiring (CAN_H, CAN_L, GND, 120Ω termination)\n");
        printf("    3. Motor ID matches? (current: %d)\n", motor_id);
        printf("    4. Bitrate = 1Mbps? (slcand -s8)\n");
        close(sock);
        return 1;
    }

    // ── Step 3: Read Parameters ────────────────────────────
    if (g_running) {
        printf("\n[3] Reading Parameters...\n");
        float val;

        flushRx(sock);
        readParam(sock, motor_id, RS_PARAM_MECH_POS, val, "mech_pos (rad)");

        flushRx(sock);
        readParam(sock, motor_id, RS_PARAM_MECH_VEL, val, "mech_vel (rad/s)");

        flushRx(sock);
        readParam(sock, motor_id, RS_PARAM_VBUS, val, "vbus (V)");
    }

    // ── Step 4: Position Control (if move_deg != 0) ────────
    if (g_running && std::abs(move_deg) > 0.1) {
        printf("\n[4] Position Control Test...\n");

        // Read starting position
        float start_pos = 0.0f;
        flushRx(sock);
        readParam(sock, motor_id, RS_PARAM_MECH_POS, start_pos, "start_pos");

        // Setup position mode
        setupPositionMode(sock, motor_id, 5.0f);

        float target_rad = start_pos + move_deg * M_PI / 180.0;

        // Move to target
        printf("\n  ⚠ 모터가 움직입니다! (%.1f° → %.1f°)\n",
               start_pos * 180.0 / M_PI, target_rad * 180.0 / M_PI);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        moveToPosition(sock, motor_id, target_rad);

        if (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // Return to start
            printf("\n  Returning to start position (%.1f°)...\n",
                   start_pos * 180.0 / M_PI);
            moveToPosition(sock, motor_id, start_pos);
        }
    } else if (std::abs(move_deg) <= 0.1) {
        printf("\n[4] Skipping movement (move_deg=0)\n");
    }

    // ── Step 5: Disable ────────────────────────────────────
    printf("\n[5] Disabling motor...\n");
    testRobstrideDisable(sock, motor_id);

    // ── Step 6: Continuous monitor (optional) ──────────────
    if (g_running) {
        printf("\n[6] Monitoring CAN bus (Ctrl+C to stop)...\n");
        printf("    Showing all received frames:\n\n");

        int count = 0;
        while (g_running) {
            uint32_t rx_id; uint8_t rx_data[8], rx_len;
            if (recvFrame(sock, &rx_id, rx_data, &rx_len, 100)) {
                printf("    [%4d] ID=0x%08X DLC=%d Data: ", count++, rx_id, rx_len);
                printHex(rx_data, rx_len);
                printf("\n");
            }
        }
    }

    printf("\n[DONE] Test complete.\n");
    close(sock);
    return 0;
}
