#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("z_plan_movegroup");

int main(int argc, char** argv)
{
  // 初始化 ROS2
  rclcpp::init(argc, argv);
  
  // 创建节点
  auto node = std::make_shared<rclcpp::Node>(
    "z_plan_movegroup",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  
  // 创建 executor 用于处理回调
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });
  
  RCLCPP_INFO(LOGGER, "========================================");
  RCLCPP_INFO(LOGGER, "Z Plan Node (MoveGroupInterface) Started!");
  RCLCPP_INFO(LOGGER, "========================================");
  
  // 等待 MoveGroup 和其他节点启动
  RCLCPP_INFO(LOGGER, "Waiting for MoveGroup to be ready...");
  rclcpp::sleep_for(std::chrono::seconds(5));
  
  try {
    // 创建 MoveGroupInterface
    RCLCPP_INFO(LOGGER, "Creating MoveGroupInterface for 'panda_arm'...");
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group = std::make_shared<MoveGroupInterface>(node, "panda_arm");
    
    // 设置规划参数
    move_group->setPlanningTime(10.0);  // 最多规划 10 秒
    move_group->setNumPlanningAttempts(10);  // 尝试 10 次
    move_group->setMaxVelocityScalingFactor(0.1);  // 速度限制为 10%
    move_group->setMaxAccelerationScalingFactor(0.1);  // 加速度限制为 10%
    
    RCLCPP_INFO(LOGGER, "✓ MoveGroupInterface created successfully!");
    
    // 获取当前末端执行器位置
    RCLCPP_INFO(LOGGER, " ");
    RCLCPP_INFO(LOGGER, "Getting current end-effector pose...");
    
    auto current_pose = move_group->getCurrentPose("panda_hand");
    
    RCLCPP_INFO(LOGGER, "📍 Current Pose:");
    RCLCPP_INFO(LOGGER, "   Position:");
    RCLCPP_INFO(LOGGER, "     x: %.4f m", current_pose.pose.position.x);
    RCLCPP_INFO(LOGGER, "     y: %.4f m", current_pose.pose.position.y);
    RCLCPP_INFO(LOGGER, "     z: %.4f m", current_pose.pose.position.z);
    RCLCPP_INFO(LOGGER, "   Orientation:");
    RCLCPP_INFO(LOGGER, "     x: %.4f", current_pose.pose.orientation.x);
    RCLCPP_INFO(LOGGER, "     y: %.4f", current_pose.pose.orientation.y);
    RCLCPP_INFO(LOGGER, "     z: %.4f", current_pose.pose.orientation.z);
    RCLCPP_INFO(LOGGER, "     w: %.4f", current_pose.pose.orientation.w);
    
    // 设置目标位置：固定的绝对坐标
    RCLCPP_INFO(LOGGER, " ");
    RCLCPP_INFO(LOGGER, "Setting target pose to absolute position...");
    
    // Set a target Pose
    auto const target_pose = []{
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = 1.0;
      msg.position.x = 0.28;
      msg.position.y = -0.2;
      msg.position.z = 0.5;
      return msg;
    }();
    
    RCLCPP_INFO(LOGGER, "🎯 Target Pose (Absolute Position):");
    RCLCPP_INFO(LOGGER, "   Position:");
    RCLCPP_INFO(LOGGER, "     x: %.4f m", target_pose.position.x);
    RCLCPP_INFO(LOGGER, "     y: %.4f m", target_pose.position.y);
    RCLCPP_INFO(LOGGER, "     z: %.4f m", target_pose.position.z);
    RCLCPP_INFO(LOGGER, "   Orientation:");
    RCLCPP_INFO(LOGGER, "     w: %.4f (no rotation)", target_pose.orientation.w);
    
    move_group->setPoseTarget(target_pose, "panda_hand");
    
    // 开始规划
    RCLCPP_INFO(LOGGER, " ");
    RCLCPP_INFO(LOGGER, "========================================");
    RCLCPP_INFO(LOGGER, "Planning trajectory...");
    RCLCPP_INFO(LOGGER, "========================================");
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group->plan(plan));
    
    if (success) {
      RCLCPP_INFO(LOGGER, " ");
      RCLCPP_INFO(LOGGER, "========================================");
      RCLCPP_INFO(LOGGER, "🎉 PLANNING SUCCEEDED! 🎉");
      RCLCPP_INFO(LOGGER, "========================================");
      
      // 输出轨迹信息
      const auto& trajectory = plan.trajectory.joint_trajectory;
      RCLCPP_INFO(LOGGER, " ");
      RCLCPP_INFO(LOGGER, "📊 Trajectory Details:");
      RCLCPP_INFO(LOGGER, "   Number of waypoints: %zu", trajectory.points.size());
      RCLCPP_INFO(LOGGER, "   Planning time: %.3f seconds", plan.planning_time);
      
      if (!trajectory.points.empty()) {
        const auto& last_point = trajectory.points.back();
        double duration = last_point.time_from_start.sec + 
                         last_point.time_from_start.nanosec * 1e-9;
        RCLCPP_INFO(LOGGER, "   Total duration: %.3f seconds", duration);
        
        RCLCPP_INFO(LOGGER, " ");
        RCLCPP_INFO(LOGGER, "   Joint names:");
        for (const auto& joint_name : trajectory.joint_names) {
          RCLCPP_INFO(LOGGER, "     - %s", joint_name.c_str());
        }
        
        // 显示前 3 个轨迹点
        RCLCPP_INFO(LOGGER, " ");
        RCLCPP_INFO(LOGGER, "   First 3 waypoints:");
        size_t num_to_show = std::min(trajectory.points.size(), size_t(3));
        for (size_t i = 0; i < num_to_show; i++) {
          const auto& point = trajectory.points[i];
          double time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
          RCLCPP_INFO(LOGGER, "     [%zu] @ t=%.3f s", i, time);
        }
        
        // 显示最后一个轨迹点的关节角度
        RCLCPP_INFO(LOGGER, " ");
        RCLCPP_INFO(LOGGER, "   Final joint positions:");
        for (size_t i = 0; i < trajectory.joint_names.size(); i++) {
          RCLCPP_INFO(LOGGER, "     %s: %.4f rad (%.2f°)", 
                      trajectory.joint_names[i].c_str(),
                      last_point.positions[i],
                      last_point.positions[i] * 180.0 / M_PI);
        }
      }
      
      // 询问是否执行（实际代码中可以直接执行）
      RCLCPP_INFO(LOGGER, " ");
      RCLCPP_INFO(LOGGER, "========================================");
      RCLCPP_INFO(LOGGER, "Executing trajectory...");
      RCLCPP_INFO(LOGGER, "========================================");
      
      auto execute_result = move_group->execute(plan);
      
      if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(LOGGER, " ");
        RCLCPP_INFO(LOGGER, "========================================");
        RCLCPP_INFO(LOGGER, "✓ EXECUTION SUCCEEDED! ✓");
        RCLCPP_INFO(LOGGER, "========================================");
        
        // 验证最终位置
        rclcpp::sleep_for(std::chrono::seconds(1));
        auto final_pose = move_group->getCurrentPose("panda_hand");
        RCLCPP_INFO(LOGGER, " ");
        RCLCPP_INFO(LOGGER, "📍 Final Pose:");
        RCLCPP_INFO(LOGGER, "   Position:");
        RCLCPP_INFO(LOGGER, "     x: %.4f m", final_pose.pose.position.x);
        RCLCPP_INFO(LOGGER, "     y: %.4f m", final_pose.pose.position.y);
        RCLCPP_INFO(LOGGER, "     z: %.4f m", final_pose.pose.position.z);
        
        double z_diff = final_pose.pose.position.z - current_pose.pose.position.z;
        RCLCPP_INFO(LOGGER, "   Z displacement: %.4f m (target was 0.02m)", z_diff);
        
      } else {
        RCLCPP_ERROR(LOGGER, "❌ Execution failed! Error code: %d", execute_result.val);
      }
      
    } else {
      RCLCPP_ERROR(LOGGER, " ");
      RCLCPP_ERROR(LOGGER, "========================================");
      RCLCPP_ERROR(LOGGER, "❌ PLANNING FAILED!");
      RCLCPP_ERROR(LOGGER, "========================================");
      RCLCPP_ERROR(LOGGER, " ");
      RCLCPP_ERROR(LOGGER, "Possible reasons:");
      RCLCPP_ERROR(LOGGER, "  1. Target pose is unreachable");
      RCLCPP_ERROR(LOGGER, "  2. Target pose causes collision");
      RCLCPP_ERROR(LOGGER, "  3. IK solver cannot find solution");
      RCLCPP_ERROR(LOGGER, "  4. Joint limits violation");
      RCLCPP_ERROR(LOGGER, " ");
      RCLCPP_ERROR(LOGGER, "Suggestions:");
      RCLCPP_ERROR(LOGGER, "  - Try smaller movement (e.g., 5mm instead of 20mm)");
      RCLCPP_ERROR(LOGGER, "  - Check RViz for collision visualization");
      RCLCPP_ERROR(LOGGER, "  - Verify current pose is valid");
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(LOGGER, "Exception caught: %s", e.what());
  }
  
  RCLCPP_INFO(LOGGER, " ");
  RCLCPP_INFO(LOGGER, "========================================");
  RCLCPP_INFO(LOGGER, "Z Plan Node Shutting Down");
  RCLCPP_INFO(LOGGER, "========================================");
  
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
