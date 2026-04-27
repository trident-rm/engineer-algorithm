/**
 ******************************************************************************
 * @file    gravity_comp_debug.cpp
 * @brief   Gravity compensation debug node (serial input)
 * @details
 *          Reads joint angles from hardware via serial, computes gravity
 *          compensation with Pinocchio RNEA and generated C code, prints
 *          comparison table to terminal. Publishes /joint_states so RViz
 *          shows the real arm pose.
 *
 * @usage   ros2 launch z_plan gravity_comp_debug.launch.py
 ******************************************************************************
 */

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>
#include "utils.hpp"
#include "tools/usb_data.hpp"

extern "C" {
    void compute_gravity_torques(const float q[6], float tau[6]);
}

static const rclcpp::Logger LOGGER = rclcpp::get_logger("gravity_comp_debug");

class GravityCompDebugNode {
public:
    GravityCompDebugNode(rclcpp::Node::SharedPtr node,
                         const std::string& planning_group = "arm")
        : node_(node)
    {
        RCLCPP_INFO(LOGGER, "=== Gravity Compensation Debug Node ===");

        // MoveIt setup (for robot model / Pinocchio)
        auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, planning_group);

        for (int i = 0; i < 10; ++i) {
            robot_model_ = move_group->getRobotModel();
            if (robot_model_) break;
            RCLCPP_WARN(LOGGER, "Waiting for robot model... (%d/10)", i + 1);
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        if (!robot_model_) throw std::runtime_error("Failed to get robot model");

        auto* jmg = robot_model_->getJointModelGroup(planning_group);
        joint_names_ = jmg->getActiveJointModelNames();
        RCLCPP_INFO(LOGGER, "Robot: %zu joints", joint_names_.size());

        // Pinocchio gravity closure
        compute_gravity_pinocchio_ = make_gravity_closure(robot_model_);
        if (!compute_gravity_pinocchio_)
            throw std::runtime_error("Failed to create Pinocchio gravity closure");

        // Serial + joint state publisher
        serial_node_ = std::make_shared<SerialRxNode>();
        joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);

        RCLCPP_INFO(LOGGER, "Serial + Pinocchio + GenCode ready.");
    }

    void start() {
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&GravityCompDebugNode::timer_callback, this));
    }

private:
    void timer_callback() {
        // Read serial
        //serial_node_->read_serial_callback();
        auto raw = serial_node_->get_robo_arm_angles();
        if (raw.size() != 6) return;

        // Convert to URDF convention
        std::vector<double> angles(raw.begin(), raw.end());
        angles[1] = -angles[1];
        angles[4] = 6.28 - angles[4];

        // Publish joint states (drives robot_state_publisher -> RViz)
        sensor_msgs::msg::JointState js;
        js.header.stamp = node_->now();
        js.name = joint_names_;
        js.position = angles;
        joint_state_pub_->publish(js);

        // Pinocchio torques
        auto pin = compute_gravity_pinocchio_(angles);

        // Generated C torques
        float q_f[6], tau_f[6];
        for (int i = 0; i < 6; ++i) q_f[i] = static_cast<float>(angles[i]);
        compute_gravity_torques(q_f, tau_f);

        // Print at ~1 Hz
        if (++print_counter_ >= 20) {
            print_counter_ = 0;
            RCLCPP_INFO(LOGGER, "--- Torque comparison (Nm) ---");
            RCLCPP_INFO(LOGGER, "Joint | Pinocchio | GenCode  |  Diff");
            for (int i = 0; i < 6; ++i) {
                double p = (i < static_cast<int>(pin.size())) ? pin[i] : 0.0;
                double g = tau_f[i];
                RCLCPP_INFO(LOGGER, "  J%d  | %+7.3f   | %+7.3f  | %+7.4f",
                           i + 1, p, g, p - g);
            }
            RCLCPP_INFO(LOGGER, "Angles: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                       angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]);
        }
    }

    rclcpp::Node::SharedPtr node_;
    moveit::core::RobotModelConstPtr robot_model_;
    std::vector<std::string> joint_names_;
    std::function<std::vector<float>(const std::vector<double>&)> compute_gravity_pinocchio_;
    std::shared_ptr<SerialRxNode> serial_node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int print_counter_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("gravity_comp_debug", opts);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::atomic<bool> should_stop{false};
    std::thread exec_thread([&executor, &should_stop]() {
        while (!should_stop && rclcpp::ok())
            executor.spin_some(std::chrono::milliseconds(10));
    });

    RCLCPP_INFO(rclcpp::get_logger("main"), "Waiting for MoveGroup...");
    rclcpp::sleep_for(std::chrono::seconds(5));

    auto debug_node = std::make_shared<GravityCompDebugNode>(node);
    debug_node->start();

    RCLCPP_INFO(rclcpp::get_logger("main"), "Running. Ctrl+C to stop.");

    while (rclcpp::ok())
        rclcpp::sleep_for(std::chrono::milliseconds(100));

    should_stop = true;
    if (exec_thread.joinable()) exec_thread.join();
    rclcpp::shutdown();
    return 0;
}
