/**
 ******************************************************************************
 * @file    test_gravity_comp.cpp
 * @brief   重力补偿对比测试 — Pinocchio RNEA vs 生成的 C 代码
 * @details
 *          扫描各种关节角组合，对比两种实现的力矩输出。
 *          不依赖串口/硬件，仅需 MoveGroupInterface 获取机器人模型。
 *
 * @usage   ros2 launch z_plan test_gravity_comp.launch.py
 ******************************************************************************
 */

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cmath>
#include <cstdio>
#include <random>
#include <algorithm>
#include <numeric>
#include "utils.hpp"

// Generated C gravity compensation (forward declare to avoid include path issues)
extern "C" {
    void compute_gravity_torques(const float q[6], float tau[6]);
}

static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_gravity_comp");
static constexpr int NUM_JOINTS = 6;
static constexpr double PASS_THRESHOLD = 0.01;  // Nm

struct TestResult {
    std::string name;
    std::vector<double> angles;
    std::vector<float> pin_tau;
    float gen_tau[NUM_JOINTS];
    double max_diff;
    bool passed;
};

static TestResult run_one(
    const std::string& name,
    const std::vector<double>& angles,
    const std::function<std::vector<float>(const std::vector<double>&)>& pin_closure)
{
    TestResult r;
    r.name = name;
    r.angles = angles;

    // Pinocchio
    r.pin_tau = pin_closure(angles);

    // Generated C
    float q_f[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; ++i)
        q_f[i] = static_cast<float>(angles[i]);
    compute_gravity_torques(q_f, r.gen_tau);

    // Compare
    r.max_diff = 0.0;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        double diff = std::abs(static_cast<double>(r.pin_tau[i]) - static_cast<double>(r.gen_tau[i]));
        if (diff > r.max_diff) r.max_diff = diff;
    }
    r.passed = (r.max_diff < PASS_THRESHOLD);
    return r;
}

static void print_result(const TestResult& r) {
    // Angles
    printf("  Test: %-20s [", r.name.c_str());
    for (int i = 0; i < NUM_JOINTS; ++i)
        printf("%s%+6.3f", i ? ", " : "", r.angles[i]);
    printf("]\n");

    // Per-joint
    for (int i = 0; i < NUM_JOINTS; ++i) {
        double diff = std::abs(static_cast<double>(r.pin_tau[i]) - static_cast<double>(r.gen_tau[i]));
        printf("    J%d: Pin=%+8.4f  Gen=%+8.4f  Diff=%+.4f\n",
               i + 1,
               static_cast<double>(r.pin_tau[i]),
               static_cast<double>(r.gen_tau[i]),
               diff);
    }
    printf("    Max diff: %.4f -> %s\n\n", r.max_diff, r.passed ? "PASS" : "FAIL");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "test_gravity_comp",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    RCLCPP_INFO(LOGGER, "=== Gravity Compensation: Pinocchio vs Generated C ===");

    // Get robot model via MoveGroupInterface
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node, "arm");

    moveit::core::RobotModelConstPtr robot_model;
    for (int i = 0; i < 10; ++i) {
        robot_model = move_group->getRobotModel();
        if (robot_model) break;
        RCLCPP_WARN(LOGGER, "Waiting for robot model... (%d/10)", i + 1);
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
    if (!robot_model) {
        RCLCPP_ERROR(LOGGER, "Failed to get robot model");
        rclcpp::shutdown();
        return 1;
    }

    // Build Pinocchio closure
    auto pin_closure = make_gravity_closure(robot_model);
    if (!pin_closure) {
        RCLCPP_ERROR(LOGGER, "Failed to build Pinocchio gravity closure");
        rclcpp::shutdown();
        return 1;
    }

    printf("\n=== Gravity Compensation: Pinocchio vs Generated C ===\n\n");

    std::vector<TestResult> results;

    // --- 1. All zeros ---
    results.push_back(run_one("all_zeros", {0, 0, 0, 0, 0, 0}, pin_closure));

    // --- 2. Single-joint sweeps ---
    const std::vector<double> sweep_vals = {-1.5, -0.5, 0.5, 1.0, 1.5};
    for (int j = 0; j < NUM_JOINTS; ++j) {
        for (double val : sweep_vals) {
            std::vector<double> angles(NUM_JOINTS, 0.0);
            angles[j] = val;
            char name[64];
            snprintf(name, sizeof(name), "J%d=%.1f", j + 1, val);
            results.push_back(run_one(name, angles, pin_closure));
        }
    }

    // --- 3. Typical poses ---
    const std::vector<std::pair<std::string, std::vector<double>>> typical = {
        {"typical_1", {0.0, -0.5, 1.0, 0.0, 0.5, 0.0}},
        {"typical_2", {0.3, 1.5, -0.5, 1.5, 1.5, 0.0}},
        {"typical_3", {-1.0, 0.8, -1.2, 0.5, -0.3, 1.0}},
        {"typical_4", {0.5, -1.0, 0.5, -1.0, 1.0, -0.5}},
    };
    for (auto& [name, angles] : typical) {
        results.push_back(run_one(name, angles, pin_closure));
    }

    // --- 4. Random poses ---
    std::mt19937 rng(42);  // fixed seed for reproducibility
    std::uniform_real_distribution<double> dist(-M_PI, M_PI);
    for (int i = 0; i < 50; ++i) {
        std::vector<double> angles(NUM_JOINTS);
        for (int j = 0; j < NUM_JOINTS; ++j)
            angles[j] = dist(rng);
        char name[64];
        snprintf(name, sizeof(name), "random_%02d", i + 1);
        results.push_back(run_one(name, angles, pin_closure));
    }

    // --- Print all results ---
    for (auto& r : results)
        print_result(r);

    // --- Summary ---
    int passed = 0;
    double global_max_diff = 0.0;
    for (auto& r : results) {
        if (r.passed) ++passed;
        if (r.max_diff > global_max_diff) global_max_diff = r.max_diff;
    }

    int total = static_cast<int>(results.size());
    printf("=== Summary: %d/%d PASSED (max diff across all: %.4f Nm) ===\n",
           passed, total, global_max_diff);

    bool all_passed = (passed == total);
    if (all_passed) {
        RCLCPP_INFO(LOGGER, "ALL TESTS PASSED");
    } else {
        RCLCPP_ERROR(LOGGER, "%d/%d TESTS FAILED", total - passed, total);
    }

    rclcpp::shutdown();
    return all_passed ? 0 : 1;
}
