/**
 * ============================================================================
 * test_integrated — RMD + Robstride 통합 모터 테스트 (ROS 없이)
 * ============================================================================
 *
 * 기능:
 *   1) N개 RMD 모터 + M개 Robstride 모터를 동일 USBCAN에서 통합 테스트
 *   2) 모든 모터 Enable + 상태 읽기
 *   3) 순차적 위치 제어 (각 모터 +30° → 원위치)
 *   4) 동시 위치 제어 (전체 +30° → 원위치)
 *   5) 안전 정리 (Robstride Disable)
 *
 * 사용법:
 *   sudo ./test_integrated [rmd_count] [rs_count] [can_channel]
 *   sudo ./test_integrated                ← 4 RMD + 2 Robstride, ch0
 *   sudo ./test_integrated 4 2 0          ← 4 RMD(1-4) + 2 Robstride(1-2), ch0
 *   sudo ./test_integrated 4 1 0          ← 4 RMD(1-4) + 1 Robstride(1), ch0
 *   sudo ./test_integrated 2 2 0          ← 2 RMD(1-2) + 2 Robstride(1-2), ch0
 *
 * 빌드 (C++/build/ 에서):
 *   cmake .. && make test_integrated
 *
 * ============================================================================
 */

#include "arm_sdk/integrated_driver.hpp"

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <cmath>
#include <chrono>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static volatile bool g_running = true;
void signalHandler(int) { g_running = false; }

// ════════════════════════════════════════════════════════════════
//  Display helpers
// ════════════════════════════════════════════════════════════════

static void printJointTable(arm_sdk::IntegratedDriver& drv)
{
    printf("┌──────────┬──────────┬──────────┬──────────┬──────────┬────────┐\n");
    printf("│  Joint   │   Type   │ Pos(deg) │ Vel(d/s) │ Eff(Nm)  │ Temp°C │\n");
    printf("├──────────┼──────────┼──────────┼──────────┼──────────┼────────┤\n");

    for (size_t i = 0; i < drv.jointCount(); ++i) {
        auto& j = drv.joint(i);
        printf("│ %-8s │ %-8s │ %8.2f │ %8.2f │ %8.3f │ %5.1f  │\n",
               j.name.c_str(),
               arm_sdk::motorTypeString(j.motor_type),
               j.position_rad * 180.0 / M_PI,
               j.velocity_rads * 180.0 / M_PI,
               j.effort_nm,
               j.temperature);
    }

    printf("└──────────┴──────────┴──────────┴──────────┴──────────┴────────┘\n");
}

static void printCompactState(arm_sdk::IntegratedDriver& drv)
{
    for (size_t i = 0; i < drv.jointCount(); ++i) {
        auto& j = drv.joint(i);
        printf("%s=%.1f° ", j.name.c_str(), j.position_rad * 180.0 / M_PI);
    }
}

// ════════════════════════════════════════════════════════════════
//  Wait for motors to settle
// ════════════════════════════════════════════════════════════════

static bool waitForSettle(arm_sdk::IntegratedDriver& drv,
                          double tolerance_deg, int timeout_ms)
{
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);
    double tol_rad = tolerance_deg * M_PI / 180.0;

    while (g_running && std::chrono::steady_clock::now() < deadline) {
        drv.readAll();

        bool all_settled = true;
        for (size_t i = 0; i < drv.jointCount(); ++i) {
            auto& j = drv.joint(i);
            double error = std::abs(j.position_rad - j.position_command);
            if (error > tol_rad) {
                all_settled = false;
            }
        }

        printf("\r  ");
        printCompactState(drv);
        fflush(stdout);

        if (all_settled) {
            printf("\n");
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    printf("\n");
    return false;
}

// ════════════════════════════════════════════════════════════════
//  Main
// ════════════════════════════════════════════════════════════════

int main(int argc, char* argv[])
{
    signal(SIGINT, signalHandler);

    int rmd_count   = (argc >= 2) ? std::atoi(argv[1]) : 4;
    int rs_count    = (argc >= 3) ? std::atoi(argv[2]) : 2;
    int can_channel = (argc >= 4) ? std::atoi(argv[3]) : 0;

    if (rmd_count < 0 || rmd_count > 6 || rs_count < 0 || rs_count > 4) {
        printf("Usage: %s [rmd_count(0-6)] [rs_count(0-4)] [can_channel]\n", argv[0]);
        return 1;
    }

    int total = rmd_count + rs_count;

    printf("╔═══════════════════════════════════════════════════════╗\n");
    printf("║  Integrated Motor Test                                ║\n");
    printf("║  %d RMD(ID 1-%d) + %d Robstride(ID 1-%d) on CAN ch%d    ║\n",
           rmd_count, rmd_count, rs_count, rs_count, can_channel);
    printf("║  Total: %d motors                                     ║\n", total);
    printf("╚═══════════════════════════════════════════════════════╝\n\n");

    // ── Configure ──────────────────────────────────────────
    arm_sdk::IntegratedDriverConfig config;
    config.device_type   = arm_sdk::USBCAN_II;
    config.device_index  = 0;
    config.can_channel   = static_cast<uint32_t>(can_channel);
    config.baudrate      = 1000000;
    config.lib_path      = "libusbcan.so";

    // RMD settings
    config.rmd_acceleration    = 500;
    config.rmd_max_vel_dps     = 360.0;
    config.rmd_default_vel_dps = 50.0;

    // Robstride settings
    config.rs_limit_speed  = 5.0f;   // rad/s

    // Add RMD joints (ID 1 ~ rmd_count)
    for (int i = 0; i < rmd_count; ++i) {
        arm_sdk::IntegratedDriverConfig::JointDef jd;
        jd.name            = "rmd_" + std::to_string(i + 1);
        jd.motor_type      = arm_sdk::MotorType::RMD;
        jd.motor_id        = i + 1;
        jd.torque_constant = 0.32;
        config.joints.push_back(jd);
    }

    // Add Robstride joints (ID 1 ~ rs_count)
    for (int i = 0; i < rs_count; ++i) {
        arm_sdk::IntegratedDriverConfig::JointDef jd;
        jd.name           = "rs_" + std::to_string(i + 1);
        jd.motor_type     = arm_sdk::MotorType::ROBSTRIDE;
        jd.motor_id       = i + 1;
        jd.max_speed_rads = config.rs_limit_speed;
        config.joints.push_back(jd);
    }

    // ── Create driver ──────────────────────────────────────
    arm_sdk::IntegratedDriver driver;

    driver.setLogCallback([](int level, const std::string& msg) {
        const char* tag = (level == arm_sdk::LOG_ERROR) ? "ERR" :
                          (level == arm_sdk::LOG_WARN)  ? "WRN" : "INF";
        printf("[%s] %s\n", tag, msg.c_str());
    });

    // ════════════════════════════════════════════════════════
    // Step 1: Configure
    // ════════════════════════════════════════════════════════
    printf("═══ Step 1: Configure ═══════════════════════════════\n");
    if (!driver.configure(config)) {
        printf("\n[FAIL] configure() failed!\n");
        printf("  1. USBCAN connected? (lsusb | grep 0471)\n");
        printf("  2. Motor power ON?\n");
        printf("  3. CAN wiring correct? (CAN_H, CAN_L, GND, 120Ω)\n");
        printf("  4. Motor IDs correct?\n");
        return 1;
    }
    printf("\n");

    if (!g_running) { driver.cleanup(); return 0; }

    // ════════════════════════════════════════════════════════
    // Step 2: Activate
    // ════════════════════════════════════════════════════════
    printf("═══ Step 2: Activate ════════════════════════════════\n");
    if (!driver.activate()) {
        printf("[FAIL] activate() failed\n");
        driver.cleanup();
        return 1;
    }
    printf("\n");

    if (!g_running) { driver.cleanup(); return 0; }

    // ════════════════════════════════════════════════════════
    // Step 3: Read all states
    // ════════════════════════════════════════════════════════
    printf("═══ Step 3: Read All Motor States ═══════════════════\n");
    driver.readAll();
    printJointTable(driver);
    printf("\n");

    if (!g_running) { driver.cleanup(); return 0; }

    // ════════════════════════════════════════════════════════
    // Step 4: Sequential test — move each motor +30°
    // ════════════════════════════════════════════════════════
    printf("═══ Step 4: Sequential Control (+30° each) ══════════\n");
    printf("  ⚠ 모터가 하나씩 순서대로 움직입니다!\n\n");

    double move_rad = 30.0 * M_PI / 180.0;  // +30°

    // Save starting positions
    std::vector<double> start_positions(driver.jointCount());
    for (size_t i = 0; i < driver.jointCount(); ++i) {
        start_positions[i] = driver.joint(i).position_rad;
    }

    for (size_t i = 0; i < driver.jointCount() && g_running; ++i) {
        auto& j = driver.joint(i);
        double target = start_positions[i] + move_rad;

        printf("  [%zu/%zu] Moving '%s' (%s) → %.1f°...\n",
               i + 1, driver.jointCount(), j.name.c_str(),
               arm_sdk::motorTypeString(j.motor_type),
               target * 180.0 / M_PI);

        j.position_command = target;
        driver.writeMotor(i);

        // Wait for this motor to settle (3° tolerance, 3s timeout)
        bool settled = waitForSettle(driver, 3.0, 3000);
        if (settled) {
            printf("  [OK] '%s' reached target\n", j.name.c_str());
        } else {
            printf("  [WARN] '%s' did not fully settle\n", j.name.c_str());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    printf("\n");

    if (!g_running) { driver.cleanup(); return 0; }

    // ════════════════════════════════════════════════════════
    // Step 5: Return all to start positions
    // ════════════════════════════════════════════════════════
    printf("═══ Step 5: Return to Start Positions ═══════════════\n");

    for (size_t i = 0; i < driver.jointCount(); ++i) {
        driver.joint(i).position_command = start_positions[i];
    }
    driver.writeAll();

    printf("  Waiting for all motors to return...\n");
    waitForSettle(driver, 3.0, 5000);

    driver.readAll();
    printJointTable(driver);
    printf("\n");

    if (!g_running) { driver.cleanup(); return 0; }

    // ════════════════════════════════════════════════════════
    // Step 6: Simultaneous movement (+30° all at once)
    // ════════════════════════════════════════════════════════
    printf("═══ Step 6: Simultaneous Control (+30° all) ═════════\n");
    printf("  ⚠ 모든 모터가 동시에 움직입니다!\n\n");

    // Read current positions as new baseline
    driver.readAll();
    for (size_t i = 0; i < driver.jointCount(); ++i) {
        start_positions[i] = driver.joint(i).position_rad;
        driver.joint(i).position_command = start_positions[i] + move_rad;
    }
    driver.writeAll();

    bool settled = waitForSettle(driver, 3.0, 5000);
    if (settled) {
        printf("  [OK] All motors reached targets\n");
    } else {
        printf("  [WARN] Some motors may not have fully settled\n");
    }

    driver.readAll();
    printJointTable(driver);
    printf("\n");

    // Return
    printf("  Returning to start...\n");
    for (size_t i = 0; i < driver.jointCount(); ++i) {
        driver.joint(i).position_command = start_positions[i];
    }
    driver.writeAll();
    waitForSettle(driver, 3.0, 5000);
    printf("\n");

    if (!g_running) { driver.cleanup(); return 0; }

    // ════════════════════════════════════════════════════════
    // Step 7: Continuous monitoring
    // ════════════════════════════════════════════════════════
    printf("═══ Step 7: Continuous Read (Ctrl+C to stop) ════════\n");

    int cycle = 0;
    while (g_running) {
        driver.readAll();

        if (cycle % 10 == 0) {
            printf("\r  [%4d] ", cycle);
            printCompactState(driver);
            fflush(stdout);
        }

        cycle++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("\n\n");

    // ════════════════════════════════════════════════════════
    // Cleanup
    // ════════════════════════════════════════════════════════
    printf("═══ Cleanup ═════════════════════════════════════════\n");
    driver.cleanup();
    printf("[DONE] Integrated test complete.\n");

    return 0;
}
