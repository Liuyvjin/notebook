# MotionPlanning Pipeline教程

#### 目录

[TOC]

## 一、运动规划管道简介

在`MoveIt!`中，运动规划器用于规划路径。然而，有时我们可能想要在运动规划前进行预处理，或者对规划得到的路径进行后处理（例如，进行时间参数化）。在这种情况下，我们使用规划管道，它将运动规划器与预处理和后处理阶段联系起来，就像一个联通的管道。

```mermaid
graph LR
	a(预处理)-->b(运动规划)
	b-->c(后处理)
```

这些可以接入管道中的预处理和后处理工具在`ROS`被成为`planning request adapters`，即`规划请求适配器`。可以通过ROS参数服务器的参数来配置它们。

在本教程中，我们将引导您完成C ++代码以实例化并调用此类规划管道，创建三条首尾相连的轨迹。

## 二、启动程序

```shell
roslaunch moveit_tutorials motion_planning_pipeline_tutorial.launch
```

**launch文件**

规划请求适配器需要在规划节点的命名空间中定义。参数名为`request_adapters`，如launch文件所示。

```xml
<launch>
  <!-- 变量定义 -->
  <arg name="debug" default="false" />
  <arg unless="$(arg debug)" name="launch_prefix" value="" />
  <arg     if="$(arg debug)" name="launch_prefix" value="gdb --ex run --args" />
  <arg name="planning_plugin" value="ompl_interface/OMPLPlanner" />
    <!-- 规划请求适配器变量，在启动node时使用 -->
  <arg name="planning_adapters" value="default_planner_request_adapters/AddTimeParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints" />
  
    <!-- 常规项目 -->
  <include file="$(find panda_moveit_config)/launch/planning_context.launch">
    <arg name="load_robot_description" value="true"/>
  </include>
  <node pkg="tf2_ros" type="static_transform_publisher" name="virtual_joint_broadcaster_0" args="0 0 0 0 0 0 odom_combined base_footprint 100" />
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="/use_gui" value="true"/>
  </node>
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true" output="screen" />
  <include file="$(find panda_moveit_config)/launch/moveit_rviz.launch">
  </include>
    
  <!-- 启动教程节点 -->
  <node name="motion_planning_pipeline_tutorial" pkg="moveit_tutorials" type="motion_planning_pipeline_tutorial" respawn="false" launch-prefix="$(arg launch_prefix)" output="screen">
    <!-- 规划器种类 -->
    <param name="planning_plugin" value="$(arg planning_plugin)" />
    <!-- 规划请求适配器 -->
    <param name="request_adapters" value="$(arg planning_adapters)" />
    <param name="start_state_max_bounds_error" value="0.1" />
  </node>

  <rosparam command="load" file="$(find panda_moveit_config)/config/kinematics.yaml"/>
  <rosparam command="load" file="$(find panda_moveit_config)/config/ompl_planning.yaml"/>

</launch>
```

## 三、代码详解

[参考代码](<https://github.com/liuyvjin/moveit_tutorials/blob/kinetic-devel/doc/motion_planning_pipeline/src/motion_planning_pipeline_tutorial.cpp>)

### 1. 加载`RobotModel`和`PlanningScene`

```cpp
// 模型加载器
robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
// 用指针指向机器人模型
robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
// 指针指向规划场景
planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
```

### 2. 创建`PlanningPipeline`对象

现在，我们可以设置`PlanningPipeline`对象，它将使用ROS参数服务器来确定要使用的请求适配器集和planning插件。

```cpp
// 对应与参数服务器中的"planning_plugin", "request_adapters"字段
planning_pipeline::PlanningPipelinePtr planning_pipeline(
    new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));
```

### 3. 可视化

```cpp
namespace rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
visual_tools.deleteAllMarkers();
/* Remote control 让使用者可以通过rviz的按钮或快捷键控制程序暂停和运行，需要配合prompt()函数 */
visual_tools.loadRemoteControl();
/* 在rviz上显示文字 */
Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
text_pose.translation().z() = 1.75;
visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XLARGE);
/* 批量发布，减少消息数量，提高效率 */
visual_tools.trigger();
/* 等待一段时间保证rviz启动完毕，这样visual_tools.prompt()输出的提示信息就不会被淹没*/
ros::Duration(10).sleep();
/* 等待用户输入，并输出提示 */
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
```

### 4. 创建规划请求（Pose）

```cpp
planning_interface::MotionPlanRequest req;
planning_interface::MotionPlanResponse res;
// 首先定义一个位姿
geometry_msgs::PoseStamped pose;
pose.header.frame_id = "panda_link0";
pose.pose.position.x = 0.3;
pose.pose.position.y = 0.0;
pose.pose.position.z = 0.75;
pose.pose.orientation.w = 1.0;
// 位置公差为0.01 m，方向公差为0.01弧度
std::vector<double> tolerance_pose(3, 0.01);
std::vector<double> tolerance_angle(3, 0.01);
// 我们将使用kinematic_constraints包中提供的函数将位姿设置为`目标约束`。
req.group_name = "panda_arm";
moveit_msgs::Constraints pose_goal =
    kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);
req.goal_constraints.push_back(pose_goal);
```

### 5. 调用规划管道

```cpp
planning_pipeline->generatePlan(planning_scene, req, res);
/* 检查规划是否成功 */
if (res.error_code_.val != res.error_code_.SUCCESS)
{
  ROS_ERROR("Could not compute plan successfully");
  return 0;
}
```

### 6. 可视化

> tip:
>
> 可视化路径使用`moveit_msgs/DisplayTrajectory`数据，可以通过不断`push_back()`来串联多条路径
>
> `DisplayTrajectory.trajectory_start`是每个关节的默认值，当路径消息中不存在某个关节值时，用默认值填充。

```cpp
ros::Publisher display_publisher =
    node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
moveit_msgs::DisplayTrajectory display_trajectory;

/* 可视化轨迹 */
ROS_INFO("Visualizing the trajectory");
moveit_msgs::MotionPlanResponse response;
res.getMessage(response);

display_trajectory.trajectory_start = response.trajectory_start;
display_trajectory.trajectory.push_back(response.trajectory);
display_publisher.publish(display_trajectory);

/* 等待用户输入，并输出提示 */
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
```

### 7.  创建规划请求（joint space）

> tip:
>
> 规划场景中的机器人当前状态的指针可以通过`planning_scene->getCurrentStateNonConst()`获得。
>
> 这个状态是这个场景规划时的起点状态。
>
> 要根据关节位姿设置起点状态的话，需要先定义`JointModelGroup`,然后使用`setJointGroupPositions` 函数

```cpp
/* 首先，将规划场景中的状态设置为最后一个规划的最终状态，这是为了创建与之前轨迹首尾相连的第二条轨迹 */
// 获得规划场景开始状态指针
robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
// 规划场景设置为上次结果的参考状态,这一步没什么用
planning_scene->setCurrentState(response.trajectory_start);
// 根据上次结果的最终关节状态设置规划场景
const robot_model::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("panda_arm");
robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
/* 设置一个关节位置目标 */
robot_state::RobotState goal_state(robot_model);
std::vector<double> joint_values = { -1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0 };
goal_state.setJointGroupPositions(joint_model_group, joint_values);
moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

req.goal_constraints.clear();
req.goal_constraints.push_back(joint_goal);
```

### 8. 调用管道规划

```cpp
planning_pipeline->generatePlan(planning_scene, req, res);
/* Check that the planning was successful */
if (res.error_code_.val != res.error_code_.SUCCESS)
{
  ROS_ERROR("Could not compute plan successfully");
  return 0;
}
/* 可视化 */
ROS_INFO("Visualizing the trajectory");
res.getMessage(response);
display_trajectory.trajectory_start = response.trajectory_start;
display_trajectory.trajectory.push_back(response.trajectory);
display_publisher.publish(display_trajectory);
/* Wait for user input */
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
```

现在你应该看到两个规划的轨迹串联在一起

### 9. 使用规划适配器

超出关节限制范围的情况，直接规划将无法进行，必须使用规划适配器进行预处理。

```cpp
/* 首先，将规划场景中的状态设置为最后一个规划的最终状态 */
robot_state = planning_scene->getCurrentStateNonConst();
planning_scene->setCurrentState(response.trajectory_start);
robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
/* 现在，设置其中一个关节稍微超出它的上限 */
const robot_model::JointModel* joint_model = joint_model_group->getJointModel("panda_joint3");
const robot_model::JointModel::Bounds& joint_bounds = joint_model->getVariableBounds();
std::vector<double> tmp_values(1, 0.0);
tmp_values[0] = joint_bounds[0].min_position_ - 0.01;
robot_state.setJointPositions(joint_model, tmp_values);
// 仍然使用之前的pose目标
req.goal_constraints.clear();
req.goal_constraints.push_back(pose_goal);
```

再次调用规划器进行规划

```cpp
  planning_pipeline->generatePlan(planning_scene, req, res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  /* 可视化轨迹 */
  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  /* 现在你应该看到三个规划的轨迹串联在一起 */
  display_publisher.publish(display_trajectory ；

  /* Wait for user input */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish the demo");

  ROS_INFO("Done");
  return 0;
}
```

