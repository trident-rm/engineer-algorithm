#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("z_plan_node");
namespace mtc = moveit::task_constructor;
using moveit::planning_interface::MoveGroupInterface;

class MTCTaskNode              
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();
  void setupPlanningScene();  // Declaration of setupPlanningScene

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("z_plan_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  // Create a PlanningSceneInterface to interact with the planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Optionally, you can add objects to the planning scene here if needed
  RCLCPP_INFO(LOGGER, "Setting up the planning scene...");

  // Wait for the planning scene to be ready
  rclcpp::sleep_for(std::chrono::seconds(2));
  
  RCLCPP_INFO(LOGGER, "Planning scene is set up.");
} 

void MTCTaskNode::doTask()
{
  // Wait a bit for the planning scene to be ready
  RCLCPP_INFO(LOGGER, "Waiting for planning scene to be ready...");
  rclcpp::sleep_for(std::chrono::seconds(3));
  
  RCLCPP_INFO(LOGGER, "Creating task...");
  task_ = createTask();

  try
  {
    RCLCPP_INFO(LOGGER, "Initializing task...");
    task_.init();
    RCLCPP_INFO(LOGGER, "Task initialized successfully!");
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  RCLCPP_INFO(LOGGER, "Planning task...");
  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  RCLCPP_INFO(LOGGER, "Planning succeeded! ✓");
  
  RCLCPP_INFO(LOGGER, "Publishing solution to RViz...");
  task_.introspection().publishSolution(*task_.solutions().front());

  RCLCPP_INFO(LOGGER, "Executing planned trajectory...");
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }
  
  RCLCPP_INFO(LOGGER, "Execution completed successfully! ✓");
  RCLCPP_INFO(LOGGER, "Task finished!");

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("z_plan_task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  // Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state for later use
#pragma GCC diagnostic pop
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sample_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interplanner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();


  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);

  // Move relative: Z+100mm
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("move z+100mm", cartesian_planner);
    stage->properties().set("marker_ns", "move_z");
    stage->properties().set("group", arm_group_name);  // Use arm group, not end_effector
    stage->properties().set("link", hand_frame);  // Reference frame for movement
    
    // Set IK frame as identity pose at end_effector
    geometry_msgs::msg::PoseStamped ik_frame;
    ik_frame.header.frame_id = hand_frame;
    ik_frame.pose.orientation.w = 1.0;
    stage->properties().set("ik_frame", ik_frame);
    
    stage->setMinMaxDistance(0.02, 0.02);  // exactly 20mm

    // Set upward direction (Z+)
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "joint";
    vec.vector.x = 0.0;
    vec.vector.y = 0.0;
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    task.add(std::move(stage));
  }

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

  RCLCPP_INFO(LOGGER, "========================================");
  RCLCPP_INFO(LOGGER, "Z Plan Node Started!");
  RCLCPP_INFO(LOGGER, "========================================");

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  RCLCPP_INFO(LOGGER, "========================================");
  RCLCPP_INFO(LOGGER, "Z Plan Node Shutting Down");
  RCLCPP_INFO(LOGGER, "========================================");
  
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
