/**
 * ============================================================================
 * test_frlab_arm — FrlabArm 6DOF SDK 하드웨어 테스트 (SocketCAN / CANable v2.0)
 * ============================================================================
 *
 * 구성: Robstride03 x2 (joint 1-2) + MyActuator X4-36 x4 (joint 3-6)
 *
 * 사전 준비:
 *   sudo slcand -o -c -s8 /dev/ttyACM0 can0
 *   sudo ip link set up can0
 *
 * 사용법:
 *   sudo ./test_frlab_arm [can_if]   # default: can0
 *
 * 테스트 순서:
 *   1) init (SocketCAN 열기, 6개 모터 확인, position mode 진입)
 *   2) 현재 상태 읽기 + 출력
 *   3) Hold loop: 현재 위치 유지 (Ctrl+C 종료)
 *
 * ============================================================================
 */

#include "arm_sdk/frlab_arm.hpp"

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <cmath>
#include <chrono>
#include <thread>
#include <array>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static volatile bool g_running = true;
void signalHandler(int) { g_running = false; }

static void printHeader()
{
    printf("┌────────┬──────────┬──────────┬──────────┬────────┐\n");
    printf("│ Joint  │ Pos(deg) │ Vel(d/s) │ Eff(Nm)  │ T(°C)  │\n");
    printf("├────────┼──────────┼──────────┼──────────┼────────┤\n");
}

static void printState(const arm_sdk::ArmState& s)
{
    const char* motor_labels[] = {
        "J1 RS03", "J2 RS03",
        "J3 X4  ", "J4 X4  ", "J5 X4  ", "J6 X4  "
    };
    printHeader();
    for (int i = 0; i < arm_sdk::ARM_DOF; ++i) {
        printf("│ %-6s │ %8.2f │ %8.2f │ %8.3f │ %5.1f  │\n",
               motor_labels[i],
               s.position[i]    * 180.0 / M_PI,
               s.velocity[i]    * 180.0 / M_PI,
               s.effort[i],
               s.temperature[i]);
    }
    printf("└────────┴──────────┴──────────┴──────────┴────────┘\n");
}

static void holdLoop(arm_sdk::FrlabArm& arm,
                     const std::array<double, arm_sdk::ARM_DOF>& hold_pos,
                     int period_ms = 10)
{
    std::array<double, arm_sdk::ARM_DOF> zeros;
    zeros.fill(0.0);

    arm_sdk::ArmState fb;
    int cycle = 0;

    printf("\n[Hold loop] Period=%dms  Ctrl+C to stop\n", period_ms);

    while (g_running) {
        auto t0 = std::chrono::steady_clock::now();

        arm.step(hold_pos, zeros, fb);

        if (cycle % 100 == 0) {
            printf("\r  J1=%6.1f° J2=%6.1f° J3=%6.1f° J4=%6.1f° J5=%6.1f° J6=%6.1f°",
                   fb.position[0] * 180.0 / M_PI,
                   fb.position[1] * 180.0 / M_PI,
                   fb.position[2] * 180.0 / M_PI,
                   fb.position[3] * 180.0 / M_PI,
                   fb.position[4] * 180.0 / M_PI,
                   fb.position[5] * 180.0 / M_PI);
            fflush(stdout);
        }

        ++cycle;

        auto elapsed = std::chrono::steady_clock::now() - t0;
        auto sleep_time = std::chrono::milliseconds(period_ms) - elapsed;
        if (sleep_time > std::chrono::milliseconds(0))
            std::this_thread::sleep_for(sleep_time);
    }
    printf("\n");
}

int main(int argc, char* argv[])
{
    signal(SIGINT, signalHandler);

    std::string can_if = (argc >= 2) ? argv[1] : "can0";

    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║  FrlabArm 6DOF Test (SocketCAN)                       ║\n");
    printf("║  RS03 x2 (J1,J2) + X4-36 x4 (J3-J6)  if=%s         ║\n", can_if.c_str());
    printf("╚═══════════════════════════════════════════════════════╝\n\n");

    printf("═══ Step 1: Init ════════════════════════════════════\n");

    arm_sdk::FrlabArm arm;
    if (!arm.init(can_if)) {
        printf("\n[FAIL] init() failed. Check:\n");
        printf("  1. CANable v2.0 연결 확인\n");
        printf("  2. sudo slcand -o -c -s8 /dev/ttyACM0 can0\n");
        printf("     sudo ip link set up can0\n");
        printf("  3. Motor power ON?\n");
        printf("  4. CAN wiring + 120Ω termination?\n");
        printf("  5. Motor IDs: RS03→1,2  X4-36→1,2,3,4\n");
        return 1;
    }
    printf("[OK] All 6 motors initialized\n\n");

    if (!g_running) return 0;

    printf("═══ Step 2: Initial State ════════════════════════════\n");

    arm_sdk::ArmState state;
    arm.read(state);
    printState(state);
    printf("\n");

    if (!g_running) return 0;

    std::array<double, arm_sdk::ARM_DOF> hold_pos = state.position;

    printf("═══ Step 3: Hold at Initial Position ════════════════\n");
    holdLoop(arm, hold_pos, 10);  // 100Hz

    printf("\n═══ Shutdown ════════════════════════════════════════\n");
    arm.shutdown();
    printf("[DONE]\n");

    return 0;
}
