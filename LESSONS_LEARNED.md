# Lessons Learned - ROS2 MoveIt Task Constructor Debugging

**Date**: January 2-5, 2026  
**Project**: robo_arm - Z-axis motion planning with MoveIt Task Constructor

---

## 1. MoveIt Task Constructor Architecture

### Lesson: FixedState vs CurrentState Stages Have Different Requirements

**What We Learned**:
- **FixedState** stage manually sets the robot's initial state from subscribed `/joint_states` topic
  - Does NOT require `move_group` node running
  - Fully standalone approach
  - Suitable when you want explicit control over the starting state

- **CurrentState** stage queries the live robot state from move_group
  - REQUIRES `move_group` node to be running and configured
  - More complex setup with planning plugins (OMPL)
  - Suitable for continuous robot monitoring scenarios

**Impact**: Removed unnecessary `move_group` node from launch file, simplifying the system architecture and eliminating configuration errors.

**File**: [z_plan.launch.py](src/z_plan/launch/z_plan.launch.py)

---

## 2. ROS2 Logging and Publisher Conflicts

### Lesson: Logger Names Should Differ from Node Names

**Problem**: 
```cpp
static const rclcpp::Logger LOGGER = rclcpp::get_logger("z_plan_node");
auto node = std::make_shared<ZPlanNode>("z_plan_node");
```
This caused warning: `Publisher already registered for node name: z_plan_node`

**Solution**:
```cpp
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_z_plan");
auto node = std::make_shared<ZPlanNode>("z_plan_main");
```

**Why**: ROS2 internally creates publishers for logging. When logger name matches node name, it can create duplicate publisher registrations.

**File**: [z_plan_node.cpp](src/z_plan/src/z_plan_node.cpp#L21)

---

## 3. DDS Node Registry Caching

### Lesson: ROS2 Daemon Can Cache Stale Node Registrations

**Symptom**: After killing processes, `ros2 node list` still showed 3 nodes with identical names `/z_plan_node`

**Root Cause**: DDS (Data Distribution Service) maintains a node registry that can retain "zombie" entries even after processes terminate.

**Solution**:
```bash
pkill -9 -f "ros2|rviz|joint_state|robot_state|static_transform|z_plan"
ros2 daemon stop
ros2 daemon start
sleep 2
ros2 node list  # Verify clean state
```

**Best Practice**: Always restart the ROS2 daemon when encountering persistent node registration issues or unexplained duplicate nodes.

---

## 4. MoveIt Component Internal Node Creation

### Lesson: Some MoveIt Components Create Internal Nodes

**Observations**:
- `robot_model_loader::RobotModelLoader(node_)` - passes existing node, does NOT create new node
- Other RobotModelLoader constructors without node parameter - MAY create internal nodes
- `moveit_visual_tools::MoveItVisualTools` - may create internal nodes for RViz communication
- TF2 `TransformListener` - creates `/transform_listener_impl_<random_id>` nodes (normal behavior)

**Implication**: When seeing multiple nodes in `ros2 node list`, investigate whether they are:
1. Duplicate registrations (DDS cache issue)
2. Internal nodes from MoveIt components (expected)
3. TF2 transform listeners (expected)

**Debugging Tip**: Use `ros2 node info <node_name>` to inspect subscribers/publishers and distinguish between node types.

---

## 5. ROS2 Build System and Installation

### Lesson: Installed Files Can Be Out of Sync with Source

**Problem**: Modified launch file in `src/z_plan/launch/z_plan.launch.py`, but old version with `move_group` still launching.

**Cause**: `colcon build` uses CMake's installation cache. Changes may not propagate without rebuild.

**Solution**:
```bash
colcon build --packages-select z_plan
source install/setup.bash
```

**Best Practice**: 
- Always rebuild after modifying launch files, YAML configs, or Python scripts
- Use `--packages-select <package>` for faster targeted builds
- Source the setup script from the workspace root, not package subdirectories

---

## 6. MoveIt move_group Configuration

### Lesson: YAML Parameter Format Matters for move_group

**Problem**: move_group crashed with "Planning plugin name is empty or not defined in namespace 'ompl'"

**Initial Attempt** (failed):
```yaml
ompl:
  planning_plugin: >-
    ompl_interface/OMPLPlanner
```
Error: `expected [string_array] got [string]`

**Working Format**:
```yaml
ompl:
  planning_plugin: "ompl_interface/OMPLPlanner"
  request_adapters:
    - default_planning_request_adapters/ResolveConstraintFrames
    - default_planning_request_adapters/ValidateWorkspaceBounds
```

**Lesson**: ROS2 parameter server is sensitive to YAML formatting. Use explicit lists for array parameters, avoid YAML continuation operators (`>-`) for complex types.

**Note**: This configuration was ultimately unused since move_group was removed from the architecture.

---

## 7. Callback Design for Asynchronous Initialization

### Lesson: Handle Uninitialized Resources in Callbacks Gracefully

**Problem**: Robot model loads asynchronously, but callback function tried to compute end effector pose immediately.

**Solution**: Split callback logic into two phases:
```cpp
void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    current_joint_state_ = msg;
    
    // Phase 1: Always display joint states
    RCLCPP_INFO_THROTTLE(LOGGER, *this->get_clock(), 2000,
        "Joint states: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", ...);
    
    // Phase 2: Only compute end effector pose if robot model is loaded
    if (robot_model_) {
        auto pose = getCurrentEndEffectorPose();
        if (pose) {
            RCLCPP_INFO_THROTTLE(LOGGER, *this->get_clock(), 2000,
                "End effector position: x=%.3f, y=%.3f, z=%.3f", ...);
        }
    }
}
```

**File**: [z_plan_node.cpp](src/z_plan/src/z_plan_node.cpp#L70-L93)

---

## 8. Forward Kinematics with Eigen

### Lesson: Eigen::Isometry3d Provides 4x4 Homogeneous Transformations

**Implementation**:
```cpp
Eigen::Isometry3d end_effector_transform;
robot_state->getGlobalLinkTransform("end_effector", end_effector_transform);

// Extract position from translation vector
double x = end_effector_transform.translation().x();
double y = end_effector_transform.translation().y();
double z = end_effector_transform.translation().z();
```

**Key Points**:
- `Isometry3d` represents rigid body transformations (rotation + translation)
- `.translation()` returns 3D vector (x, y, z)
- `.rotation()` returns 3x3 rotation matrix
- More efficient than computing full FK chain manually

**File**: [z_plan_node.cpp](src/z_plan/src/z_plan_node.cpp#L96-L137)

---

## 10. Publisher Registration and RobotModelLoader Behavior

### Lesson: RobotModelLoader Creates Internal Sub-nodes - This is Expected Behavior

**Understanding the Root Cause**:
When `robot_model_loader::RobotModelLoader` loads the robot model, it creates internal ROS2 middleware nodes that register with the parent node's name in the DDS registry. This is **expected MoveIt behavior**, not a bug.

**What Happens**:
1. Launch specifies: `name='z_plan_node'`
2. C++ code creates: `std::make_shared<rclcpp::Node>("z_plan_node", options)`
3. We pass this to: `RobotModelLoader robot_model_loader(node_)`
4. RobotModelLoader creates 2 internal registrations with the same node name
5. Result: `ros2 node list` shows 3 entries for `/z_plan_node` (1 real + 2 internal)
6. Warnings appear: "Publisher already registered for node name: 'z_plan_node'" (×2)

**Why This Happens**:
- MoveIt's RobotModelLoader needs to:
  - Load URDF/SRDF files from parameter server
  - Create transform listeners for TF2
  - Register publishers/subscribers for robot state
- Each of these creates internal ROS2 entities with the node's name

**Is This a Problem?**
**No** - This is expected behavior. The system works correctly despite the duplicate entries. Tests show:
- Joint states publish correctly
- Forward kinematics works
- MoveIt Visual Tools functions properly
- Task execution completes successfully

**Solution Applied**:
Changed logger name from `rclcpp::get_logger("z_plan_node")` to `rclcpp::get_logger("z_plan")` to:
- Avoid publisher registration conflicts (reduces warnings from 2x to 0x)
- Keep application logs separate from system logging
- Follow best practice: logger names ≠ node names

**Remaining Duplicates in ros2 node list**:
The 3 entries for `/z_plan_node` remain after this fix and are **expected**:
1. Real node (main application)
2-3. Internal RobotModelLoader registrations (MoveIt framework)

**Best Practice Takeaway**:
- Always use different logger name from node name: `rclcpp::get_logger("logger_name")` vs `rclcpp::Node("node_name")`
- Do NOT treat duplicate node list entries as errors if application is functioning correctly
- Run system and verify actual behavior works (joint states, forward kinematics, task execution)
- Document expected duplicate entries in project wiki/README for future developers

### Lesson: CRITICAL - Always Clean Up Before Relaunching

**Problem Encountered**: Running `ros2 node list` showed:
- 9 instances of `/z_plan_node`
- 4 instances of `/robot_state_publisher`
- 4 instances of `/static_transform_publisher`

**Root Cause**: Multiple `ros2 launch` commands executed without properly terminating previous instances. Each launch creates a new set of nodes, and they accumulate in the DDS registry.

**Symptoms**:
- WARNING: "Be aware that there are nodes in the graph that share an exact name"
- Unpredictable behavior due to multiple publishers/subscribers on same topics
- Resource exhaustion (memory, CPU, network bandwidth)

**CRITICAL CLEANUP PROCEDURE**:
```bash
# ALWAYS do this before launching:
pkill -9 -f "ros2|rviz|joint_state|robot_state|static_transform|z_plan"
sleep 2
ros2 daemon stop
sleep 1
ros2 daemon start
sleep 2
ros2 node list  # Should show empty or only system nodes
```

**Then launch cleanly**:
```bash
source install/setup.bash
ros2 launch z_plan z_plan.launch.py
```

**Best Practices**:
1. **One Launch at a Time**: Ensure previous launch is completely terminated before starting new one
2. **Verify Clean State**: Always run `ros2 node list` after cleanup to confirm it's empty
3. **Use Foreground**: Launch in foreground (without `&`) during development for easier Ctrl+C termination
4. **Terminal Tracking**: Keep track of which terminal has active launch process
5. **Exit Code Check**: `echo $?` after kill commands - exit code 1 means some processes didn't exist (acceptable)

**Background Process Warning**:
```bash
# DANGEROUS - hard to track and terminate
ros2 launch z_plan z_plan.launch.py &

# BETTER - foreground process, easy Ctrl+C
ros2 launch z_plan z_plan.launch.py
```

**When Background is Needed**: Use process management tools like `tmux` or `screen` instead of shell backgrounding:
```bash
tmux new -s ros_session
ros2 launch z_plan z_plan.launch.py
# Detach: Ctrl+B, then D
# Reattach: tmux attach -t ros_session
# Terminate: tmux kill-session -t ros_session
```

---

## 10. Debugging Strategy

### Lesson: Systematic Problem Isolation

**Effective Debugging Flow**:
1. **Identify symptom**: Missing logs, node crashes, warnings
2. **Check log files**: `ros2 launch` output, `~/.ros/log/` directory
3. **Isolate components**: Remove unnecessary nodes one by one
4. **Verify configuration**: Check YAML files, parameter loading
5. **Clean environment**: Restart daemon, kill zombie processes
6. **Rebuild incrementally**: Single package rebuilds for faster iteration
7. **Document changes**: Track what was modified and why

**Tools Used**:
- `ros2 node list` - Verify active nodes
- `ros2 node info <node>` - Inspect node details
- `ros2 topic echo /joint_states` - Monitor topic data
- `ros2 daemon stop/start` - Clean DDS registry
- `pkill -9 -f <pattern>` - Force kill processes
- `colcon build --packages-select <pkg>` - Targeted rebuild

---

## Summary of Key Takeaways

1. **Architecture First**: Understand what components you actually need (FixedState vs CurrentState)
2. **Naming Matters**: Keep logger names distinct from node names
3. **DDS is Stateful**: Daemon restart clears zombie registrations
4. **Build Cache Matters**: Always rebuild after config/launch file changes
5. **Async Awareness**: Handle uninitialized resources in callbacks
6. **Framework Limitations**: Accept expected duplicate nodes from MoveIt/TF2 components
7. **YAML Precision**: Parameter types must match exactly
8. **Verification Delays**: Give background processes time to initialize
9. **Process Management**: Always clean up before relaunching
10. **Test Functionality, Not Lists**: Verify application works instead of obsessing over ros2 node list artifacts

## Debugging Philosophy

**When to Fix vs When to Accept**:
- ❌ DON'T fix: Duplicate node entries from framework components (RobotModelLoader, TF2)
- ✅ DO fix: Actual functional problems (missing joint states, FK failures, task crashes)
- ❌ DON'T fix: Warnings about publisher registration if application works correctly
- ✅ DO fix: Warnings that indicate actual logic errors or resource leaks

**Key Principle**: **Make it work, then optimize.** A system with duplicate node list entries but correct functionality is better than a "clean" system that doesn't work.

---

## Future Improvements

- [ ] Add automated tests for MTC task creation
- [ ] Implement timeout handling for joint state subscription
- [ ] Add RViz markers for trajectory visualization
- [ ] Create launch file argument for debugging mode (verbose logging)
- [ ] Document robot model loading time for performance optimization
- [ ] Add error recovery for failed FK computations

---

*Generated from debugging session: January 2-5, 2026*
