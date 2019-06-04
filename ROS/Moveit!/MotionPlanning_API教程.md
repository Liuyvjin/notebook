# MotionPlanning API 教程

#### 目录

[TOC]

## 一、教程简介

在`MoveIt!`中，运动规划器`motion planners `是使用插件结构加载的。这允许`MoveIt!`在运行时加载运动规划器。

## 二、启动程序

```cpp
roslaunch moveit_tutorials motion_planning_api_tutorial.launch
```

launch文件

```xml
<launch>
  <node name="motion_planning_api_tutorial" pkg="moveit_tutorials" type="motion_planning_api_tutorial" respawn="false" output="screen">
    <rosparam command="load" file="$(find panda_moveit_config)/config/kinematics.yaml"/>
    <!-- 定义规划器种类 -->
    <param name="/planning_plugin" value="ompl_interface/OMPLPlanner"/>
    <rosparam command="load" file="$(find panda_moveit_config)/config/ompl_planning.yaml"/>
  </node>
</launch>
```

## 三、程序详解

[参考程序](<https://github.com/Liuyvjin/moveit_tutorials/blob/kinetic-devel/doc/motion_planning_api/src/motion_planning_api_tutorial.cpp>)

### 1. 加载规划器

使用规划器非常简单。在`MoveIt!`中，规划器被设置为插件，您还可以使用`ROS pluginlib`接口来加载您想要使用的任何规划器。在加载规划器之前，我们需要两个对象：一个机器人模型`RobotModel`和一个规划场景`PlanningScene`。首先实例化一个`RobotModelLoader`对象，该对象将在ROS参数服务器上查找机器人描述，并构造一个供我们使用的`RobotModel`。

```cpp
const std::string PLANNING_GROUP = "panda_arm";
robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
// 创建 RobotState 和 JointModelGroup 的 `指针` 来跟踪当前机器人的姿态和规划组 
robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
// 构造 PlanningScene `指针` 指向当前规划场景
planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
```

> 我们自己新建的`RobotState`或者是`PlanningScene`只是一个数据对象，与当前机器人并没有关联，但是如果用**指针**指向当前机器人的`RobotState`或者是`PlanningScene`，我们就能随时获得机器人的信息。

通过名字构造一个加载器来加载规划器。注意，我们在这里使用的是`ROS pluginlib`库。

```cpp
// 构造加载器指针 planner_plugin_loader
boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
// 声明一个规划器指针
planning_interface::PlannerManagerPtr planner_instance;
std::string planner_plugin_name;
```

我们将从ROS参数服务器的`planning_plugin`字段，获得要加载的规划插件的名称，然后加载`planner`，并且确保捕获所有异常。

```cpp
// 从参数服务器读取规划器名字
if (!node_handle.getParam("planning_plugin", planner_plugin_name))
  ROS_FATAL_STREAM("Could not find planner plugin name");
// 尝试构造加载器
try
{
  planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
      "moveit_core", "planning_interface::PlannerManager"));
}
catch (pluginlib::PluginlibException& ex)
{
  ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
}
// 尝试加载规划器
try
{
  planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
  if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
    ROS_FATAL_STREAM("Could not initialize planner instance");
  ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
}
catch (pluginlib::PluginlibException& ex)
{
  const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
  std::stringstream ss;
  for (std::size_t i = 0; i < classes.size(); ++i)
    ss << classes[i] << " ";
  ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                       << "Available plugins: " << ss.str());
}
```

### 2. Pose 目标规划

运动规划需要两个对象，一个是请求对象`MotionPlanRequest`，另一个是响应对象`MotionPlanResponse`

```cpp
planning_interface::MotionPlanRequest req;
planning_interface::MotionPlanResponse res;
// 期望的末端位姿
geometry_msgs::PoseStamped pose;
pose.header.frame_id = "panda_link0";
pose.pose.position.x = 0.3;
pose.pose.position.y = 0.0;
pose.pose.position.z = 0.75;
pose.pose.orientation.w = 1.0;
// 容差
std::vector<double> tolerance_pose(3, 0.01);
std::vector<double> tolerance_angle(3, 0.01);
```

使用`kinematic_constraints`包中提供的功能函数可以方便地将：**位姿+容差 转换为 约束**。

```cpp
req.group_name = "panda_arm";
// 创建一个约束目标
moveit_msgs::Constraints pose_goal =
    kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);
// 将约束赋给规划请求对象
req.goal_constraints.push_back(pose_goal);
```

现在我们构造一个规划内容`Planning Context`，它封装了场景、请求和响应。我们使用这个规划内容调用规划器

```cpp
planning_interface::PlanningContextPtr context =
    planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
context->solve(res);
if (res.error_code_.val != res.error_code_.SUCCESS)
{
  ROS_ERROR("Could not compute plan successfully");
  return 0;
}
```

### 3. 可视化结果

```cpp
ros::Publisher display_publisher =
    node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
moveit_msgs::DisplayTrajectory display_trajectory;
/* 可视化轨迹 */
ROS_INFO("Visualizing the trajectory");
moveit_msgs::MotionPlanResponse response;
res.getMessage(response); // 从结果中读取数据
display_trajectory.trajectory_start = response.trajectory_start;
display_trajectory.trajectory.push_back(response.trajectory);
display_publisher.publish(display_trajectory);
/* We can also use visual_tools to wait for user input */
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
```

### 4. Joint Space 目标规划

```cpp
/* 首先，将规划场景中的状态设置为刚才规划的最终状态 */
planning_scene->setCurrentState(response.trajectory_start);
robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions); 
```

设置一个关节空间目标

> tips:
>
> 关节空间目标，是`RobotState`类型的，通常用使用`robot_model`来初始化，然后定义每个关节的值。也可以使用`current RobotState`来初始化，然后更改其中某个关节的值。

```cpp
robot_state::RobotState goal_state(robot_model);
std::vector<double> joint_values = { -1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0 };
goal_state.setJointGroupPositions(joint_model_group, joint_values);
// 构造约束
moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
req.goal_constraints.clear();  // 清除上次的约束
req.goal_constraints.push_back(joint_goal);  // 添加这次的约束
```

调用规划器，可视化

```cpp
/* 重构 planning context */
context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
/* 进行规划求解 */
context->solve(res);
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
display_publisher.publish(display_trajectory);
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

/* 我们将增加更多的目标。但首先，将机器人的状态设置为刚才规划的最终状态 */
robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

/* 回到第一个目标状态 pose_goal */
req.goal_constraints.clear();
req.goal_constraints.push_back(pose_goal);
context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
context->solve(res);
res.getMessage(response);
display_trajectory.trajectory.push_back(response.trajectory);
display_publisher.publish(display_trajectory);
/* Wait for user input */
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
```

### 5. 添加路径约束

让我们再添加一个新的Pose目标。这一次，我们还将为运动添加一个路径约束。

> tips:
>
> 注意区分规划请求的目标约束`req.goal_constraints`和路径约束`path_constraints`，两者都是`moveit_msgs::Constraints`类型，但是前者实际上指定的是求解的目标，后者是规划路径需要满足的约束。

```c++
/* 构建新的 pose goal */
pose.pose.position.x = 0.32;
pose.pose.position.y = -0.25;
pose.pose.position.z = 0.65;
pose.pose.orientation.w = 1.0;
moveit_msgs::Constraints pose_goal_2 =
    kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);
/* 首先，将机器人的状态设置为刚才规划的最终状态 */
robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
/* 添加目标约束 */
req.goal_constraints.clear();
req.goal_constraints.push_back(pose_goal_2);

/* 但是，我们对运动施加一个路径约束。
   这里，我们要求末端执行器保持水平 */
geometry_msgs::QuaternionStamped quaternion;
quaternion.header.frame_id = "panda_link0";
quaternion.quaternion.w = 1.0;
req.path_constraints = kinematic_constraints::constructGoalConstraints("panda_link8", quaternion);
```

施加路径约束要求规划器在末端执行器的工作空间内进行推理，因此，我们还需要为允许的规划空间体积指定一个界限；注意：默认边界由`WorkspaceBounds`请求适配器自动填充（这是OMPL管道的一部分，但在本例中没有使用）。我们使用一个肯定包含手臂的可达空间的边界。这很好，因为在对arm进行规划时，没有在这个体积中进行采样；边界仅用于确定抽样配置是否有效。

```cpp
req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
    req.workspace_parameters.min_corner.z = -2.0; // 工作空间最小值
req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
    req.workspace_parameters.max_corner.z = 2.0;  // 工作空间最大值
```

调用规划器，可视化到目前为止所有的规划

```cpp
context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
context->solve(res);
res.getMessage(response);
// 将新的轨迹加入
display_trajectory.trajectory.push_back(response.trajectory);
// 现在可以看到四条轨迹
display_publisher.publish(display_trajectory);
```

[回到目录](#目录)