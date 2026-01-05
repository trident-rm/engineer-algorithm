#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <memory>
#include <vector>
#include <thread>
#include <chrono>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("z_plan");
namespace mtc = moveit::task_constructor;

// MTC任务节点类
class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

private:
  // 创建MTC任务
  mtc::Task createTask();
  
  // 关节状态回调
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  
  // 获取当前末端执行器位姿
  geometry_msgs::msg::PoseStamped getCurrentEndEffectorPose();

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  std::vector<double> joint_angles_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  std::chrono::steady_clock::time_point last_callback_time_;
  const std::chrono::milliseconds callback_interval_{1000};  // 1秒间隔
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("z_plan_node", options) }
{
  // 加载机器人模型 - 直接使用主节点
  robot_model_loader::RobotModelLoader robot_model_loader(node_);
  robot_model_ = robot_model_loader.getModel();
  robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
  robot_state_->setToDefaultValues();
  
  // 初始化时间戳
  last_callback_time_ = std::chrono::steady_clock::now();
  
  // 订阅关节状态
  joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&MTCTaskNode::joint_state_callback, this, std::placeholders::_1));
  
  RCLCPP_INFO(LOGGER, "Z Plan Node started. Waiting for joint states...");
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 检查是否到了1秒间隔
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_callback_time_);
  
  if (elapsed < callback_interval_) {
    // 还没到1秒，只更新数据但不打印日志
    if (msg->position.size() >= 6)
    {
      joint_angles_.clear();
      for (size_t i = 0; i < 6; i++)
      {
        joint_angles_.push_back(msg->position[i]);
      }
    }
    return;  // 提前返回，不执行后续代码
  }
  
  // 更新时间戳
  last_callback_time_ = now;
  
  // 每秒执行一次的代码
  if (msg->position.size() >= 6)
  {
    joint_angles_.clear();
    for (size_t i = 0; i < 6; i++)
    {
      joint_angles_.push_back(msg->position[i]);
    }
    
    RCLCPP_INFO(LOGGER, 
        "Joint Angles: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
        joint_angles_[0], joint_angles_[1], joint_angles_[2],
        joint_angles_[3], joint_angles_[4], joint_angles_[5]);
    
    // 计算并显示末端执行器位置
    auto pose = getCurrentEndEffectorPose();
    RCLCPP_INFO(LOGGER,
        "End Effector Position: x=%.3f, y=%.3f, z=%.3f",
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  }
}

geometry_msgs::msg::PoseStamped MTCTaskNode::getCurrentEndEffectorPose()
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  
  if (joint_angles_.empty() || !robot_state_) {
    return pose;
  }

  // 使用MoveIt的FK计算末端执行器位姿
  const moveit::core::JointModelGroup* joint_model_group = 
      robot_model_->getJointModelGroup("arm");
  
  if (!joint_model_group) {
    RCLCPP_ERROR(LOGGER, "Joint model group 'arm' not found!");
    return pose;
  }

  // 设置关节角度
  robot_state_->setJointGroupPositions(joint_model_group, joint_angles_);
  
  // 重要：更新链接变换以重新计算FK
  robot_state_->updateLinkTransforms();
  
  // 获取末端执行器链接的全局变换
  const Eigen::Isometry3d& end_effector_state = 
      robot_state_->getGlobalLinkTransform("end_effector");
  
  // 转换为PoseStamped消息
  pose.pose.position.x = end_effector_state.translation().x();
  pose.pose.position.y = end_effector_state.translation().y();
  pose.pose.position.z = end_effector_state.translation().z();
  
  Eigen::Quaterniond q(end_effector_state.rotation());
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  return pose;
}


void MTCTaskNode::doTask()
{
  // 等待关节角度数据
  auto timeout = std::chrono::seconds(5);
  auto start = std::chrono::steady_clock::now();
  while (joint_angles_.empty() && 
         (std::chrono::steady_clock::now() - start) < timeout)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  
  if (joint_angles_.empty()) {
    RCLCPP_ERROR(LOGGER, "No joint states received. Cannot create task.");
    return;
  }

  // 创建MoveIt Visual Tools用于RViz可视化 - 直接使用主节点
  // 使用独立的命名空间发布标记数组，避免与RViz GUI面板的主题冲突
  // GUI响应来自 /rviz_visual_tools_gui 主题的命令，但我们的标记发布到 /z_plan_markers
  moveit_visual_tools::MoveItVisualTools visual_tools(
      node_, "base_link", "z_plan_markers", robot_model_);
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // 提示用户调整初始状态
  visual_tools.prompt("Press 'Next' in RViz to start planning the z+100mm movement");
  visual_tools.trigger();
  
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed: " << e);
    return;
  }

  RCLCPP_INFO(LOGGER, "Planning task...");
  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    visual_tools.prompt("Planning failed! Check the logs.");
    visual_tools.trigger();
    return;
  }
  
  RCLCPP_INFO(LOGGER, "Task planning succeeded! Found %zu solutions", task_.solutions().size());
  
  // 获取最佳解决方案
  if (task_.solutions().empty()) {
    RCLCPP_ERROR(LOGGER, "No solutions found!");
    return;
  }

  auto solution = task_.solutions().front();
  
  // 在RViz中可视化轨迹
  visual_tools.prompt("Press 'Next' to visualize the trajectory in RViz");
  visual_tools.trigger();
  task_.introspection().publishSolution(*solution);
  
  // 显示轨迹信息
  RCLCPP_INFO(LOGGER, "Solution cost: %.3f", solution->cost());
  RCLCPP_INFO(LOGGER, "Solution comment: %s", solution->comment().c_str());

  // 提示用户是否执行
  visual_tools.prompt("Press 'Next' in RViz to execute the z+100mm movement");
  visual_tools.trigger();
  
  // 执行任务
  auto result = task_.execute(*solution);
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with error code: " << result.val);
    visual_tools.prompt("Execution failed!");
    visual_tools.trigger();
    return;
  }

  RCLCPP_INFO(LOGGER, "Task execution succeeded!");
  visual_tools.prompt("Task completed successfully!");
  visual_tools.trigger();

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("Z_Plan_Task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm";
  const auto& end_effector_name = "end_effector";

  // 设置任务属性
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", end_effector_name);
  task.setProperty("ik_frame", end_effector_name);
  
  // 1. CurrentState - 获取当前状态
  auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
  task.add(std::move(current_state));

  // 获取当前末端执行器位姿（只计算一次）
  auto current_pose = getCurrentEndEffectorPose();

  // 2. Generator - 生成当前末端执行器位姿
  auto generator_stage = std::make_unique<mtc::stages::GeneratePose>("generate current pose");
  generator_stage->setPose(current_pose);
  generator_stage->properties().set("marker_ns", "current_pose");
  task.add(std::move(generator_stage));

  // 3. Propagator - 使用CartesianPath移动到目标位置 (y + 100mm)
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);  // 1cm步长
  
  auto move_to_target = std::make_unique<mtc::stages::MoveTo>("move z+100mm", cartesian_planner);
  move_to_target->setGroup(arm_group_name);
  
  // 设置目标位姿：当前位置 + z轴100mm
  auto target_pose = current_pose;  // 复用当前位姿
  target_pose.pose.position.z += 0.10;  // 100mm = 0.1m
  move_to_target->setGoal(target_pose);
  task.add(std::move(move_to_target));

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
