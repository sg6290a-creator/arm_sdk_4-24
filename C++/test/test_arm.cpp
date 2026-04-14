/**
 * ============================================================================
 * test_arm — ROS 없이 ArmDriver 통신 테스트
 * ============================================================================
 *
 * 사용법:
 *   ./test_arm [motor_count] [can_channel]
 *   ./test_arm 4 0          ← 모터 4개, CAN ch0
 *   ./test_arm 1 0          ← 모터 1개만 테스트
 *
 * 빌드 (C++/ 폴더에서):
 *   mkdir -p build && cd build
 *   cmake .. && make test_arm
 *   ./test_arm
 *
 * ============================================================================
 */

#include "arm_sdk/usbcan_device.hpp"
#include "arm_sdk/rmd_protocol.hpp"
#include "arm_sdk/arm_driver.hpp"

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <chrono>
#include <thread>

static volatile bool g_running = true;

void signalHandler(int) { g_running = false; }

void printUsage(const char* prog)
{
    printf("Usage: %s [motor_count] [can_channel]\n", prog);
    printf("  motor_count : 1~6 (default: 4)\n");
    printf("  can_channel : 0 or 1 (default: 0)\n");
    printf("\nExample:\n");
    printf("  %s 4 0    ← 4 motors on CAN channel 0\n", prog);
    printf("  %s 1 0    ← 1 motor only (ID=1)\n", prog);
}

int main(int argc, char* argv[])
{
    signal(SIGINT, signalHandler);

    int motor_count = 4;
    int can_channel = 0;

    if (argc >= 2) motor_count = std::atoi(argv[1]);
    if (argc >= 3) can_channel = std::atoi(argv[2]);

    if (motor_count < 1 || motor_count > 6) {
        printUsage(argv[0]);
        return 1;
    }

    printf("╔══════════════════════════════════════════╗\n");
    printf("║  ARM SDK Test — %d motors, CAN ch%d       ║\n", motor_count, can_channel);
    printf("╚══════════════════════════════════════════╝\n\n");

    // ── Configure ArmDriver ────────────────────────────────
    arm_sdk::ArmDriverConfig config;
    config.device_type   = arm_sdk::USBCAN_II;
    config.device_index  = 0;
    config.can_channel   = static_cast<uint32_t>(can_channel);
    config.baudrate      = 1000000;
    config.lib_path      = "libusbcan.so";
    config.motor_acceleration  = 500;
    config.max_velocity_dps    = 360.0;
    config.default_velocity_dps = 50.0;

    for (int i = 0; i < motor_count; ++i) {
        arm_sdk::ArmDriverConfig::JointDef jd;
        jd.name            = "joint_" + std::to_string(i + 1);
        jd.actuator_id     = i + 1;
        jd.torque_constant = 0.32;
        config.joints.push_back(jd);
    }

    // ── Create and configure driver ────────────────────────
    arm_sdk::ArmDriver driver;

    driver.setLogCallback([](int level, const std::string& msg) {
        const char* tag = (level == arm_sdk::LOG_ERROR) ? "ERR" :
                          (level == arm_sdk::LOG_WARN)  ? "WRN" : "INF";
        printf("[%s] %s\n", tag, msg.c_str());
    });

    printf("[TEST] Step 1: Configure (load lib → open device → init CAN → test motors)\n");
    if (!driver.configure(config)) {
        printf("\n[FAIL] configure() failed. Check:\n");
        printf("  - USBCAN device connected? (lsusb | grep 0471)\n");
        printf("  - libusbcan.so installed? (ls /lib/libusbcan.so)\n");
        printf("  - Motor power ON?\n");
        printf("  - CAN wiring correct? (CAN_H, CAN_L, GND, 120Ω termination)\n");
        return 1;
    }

    printf("\n[TEST] Step 2: Activate (read current positions)\n");
    if (!driver.activate()) {
        printf("[FAIL] activate() failed\n");
        return 1;
    }

    // ── Print initial state ────────────────────────────────
    printf("\n┌────────────────────────────────────────────────────┐\n");
    printf("│  Initial Motor States                              │\n");
    printf("├──────────┬──────────┬──────────┬──────────┬────────┤\n");
    printf("│  Joint   │ Pos(deg) │ Vel(dps) │ Eff(Nm)  │ Temp°C │\n");
    printf("├──────────┼──────────┼──────────┼──────────┼────────┤\n");
    for (size_t i = 0; i < driver.jointCount(); ++i) {
        auto& j = driver.joint(i);
        printf("│ %-8s │ %8.2f │ %8.2f │ %8.3f │ %4d   │\n",
               j.joint_name.c_str(),
               arm_sdk::RMDProtocol::radToDeg(j.position_state),
               arm_sdk::RMDProtocol::radSToDps(j.velocity_state),
               j.effort_state,
               j.temperature);
    }
    printf("└──────────┴──────────┴──────────┴──────────┴────────┘\n");

    // ── Continuous read loop ───────────────────────────────
    printf("\n[TEST] Step 3: Continuous read (Ctrl+C to stop)\n");
    printf("  Reading all motors every 100ms...\n\n");

    int cycle = 0;
    while (g_running) {
        driver.readAll();

        if (cycle % 10 == 0) {  // Print every 1 second
            printf("\r[%4d] ", cycle);
            for (size_t i = 0; i < driver.jointCount(); ++i) {
                auto& j = driver.joint(i);
                printf("J%zu=%.1f° ", i + 1,
                       arm_sdk::RMDProtocol::radToDeg(j.position_state));
            }
            fflush(stdout);
        }

        cycle++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    printf("\n\n[TEST] Ctrl+C received. Cleaning up...\n");
    driver.cleanup();
    printf("[DONE] Test complete.\n");

    return 0;
}
