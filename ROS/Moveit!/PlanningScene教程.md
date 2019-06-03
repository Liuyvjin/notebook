# PlanningScene教程

#### 目录

[TOC]

## 一、启动程序

`PlanningScene`类提供了用于冲突检查和约束检查的主接口。在本教程中，我们将探索该类的c++接口。

启动程序使用：

```cpp
roslaunch moveit_tutorials planning_scene_tutorial.launch
```



## 二、程序详解

[参考程序](<https://github.com/Liuyvjin/moveit_tutorials/blob/kinetic-devel/doc/planning_scene/src/planning_scene_tutorial.cpp>)

### 1. 建立（Setup）

`PlanningScene`类可以使用`RobotModel`或`URDF+SRDF`轻松建立和配置。然而，这不是实例化PlanningScene的推荐方法。`PlanningSceneMonitor`是使用机器人关节和传感器上的数据创建和维护当前规划场景的推荐方法(在下一教程中详细讨论)。在本教程中，我们将直接实例化一个PlanningScene类，但是这种实例化方法仅用于演示。

头文件

```cpp
#include <moveit/planning_scene/planning_scene.h>
```

实例化`PlanningScene`

```cpp
robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
// 构造函数需要传入一个`RobotModelPtr`
planning_scene::PlanningScene planning_scene(kinematic_model);
```

### 2. 碰撞检测

#### 2.1 自碰撞检测

我们要做的第一件事就是检查机器人当前的状态是否处于自碰撞状态，即机器人当前的配置是否会导致机器人的各个部件相互碰撞。为此，我们将构造一个`CollisionRequest`对象和一个`CollisionResult`对象，并将它们传递给冲突检查函数。注意，机器人是否处于自碰撞的结果包含在结果对象中。自碰撞检测使用了一个未填充的机器人版本，也就是说，它直接使用URDF中提供的没有额外填充的碰撞网格（collision meshes）。

```cpp
collision_detection::CollisionRequest collision_request; // 请求对象
collision_detection::CollisionResult collision_result;  // 结果对象
// 碰撞检测，若碰撞collision_result.collision为true
planning_scene.checkSelfCollision(collision_request, collision_result);
ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
```

改变当前状态，再次检查是否碰撞；注意再次检查之前要清除上次检测的结果

```cpp
robot_state::RobotState& current_state= planning_scene.getCurrentStateNonConst();  // 获得当前机器人状态的引用
current_state.setToRandomPositions();  // 将状态设置为随机值
collision_result.clear();  // 清除上次碰撞检测结果
planning_scene.checkSelfCollision(collision_request, collision_result);
ROS_INFO_STREAM("Test 2: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
```

#### 2.2 规划组碰撞检测

现在，我们只会对熊猫机器人的手进行碰撞检查，也就是说，我们会检查机器人的手和身体的其他部分是否有碰撞。我们可以通过将规划组名“hand”添加到冲突请求中来明确地要求这样做。

```cpp
collision_request.group_name = "hand";  // 将规划组名添加到请求对象中
current_state.setToRandomPositions();  
collision_result.clear();
planning_scene.checkSelfCollision(collision_request, collision_result);
ROS_INFO_STREAM("Test 3: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
```

#### 2.3 获得联系信息

对于联系信息，我的理解是，对于每一个碰撞，它发生在哪两个link之间，就是联系信息。

首先，手动设置熊猫臂的位置，保证内部(自我)碰撞确实发生。注意，这个状态现在实际上已经超出了熊猫的关节极限，这一点我们也可以直接检查。

```cpp
// 手动选择的关节值
std::vector<double> joint_values = { 0.0, 0.0, 0.0, -2.9, 0.0, 1.4, 0.0 };
const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("panda_arm");
// 设置关节值
current_state.setJointGroupPositions(joint_model_group, joint_values);
// 检查是否符合关节限制：RobotState.satisfiesBounds()
ROS_INFO_STREAM("Test 4: Current state is "
                << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));
```

现在，我们可以获得任何在这个设置下发生的碰撞的联系信息。我们可以通过在请求对象中填入适当的字段，并设置函数返回的联系数量的最大值。

```cpp
collision_request.contacts = true;  // 开启返回联系信息的功能
collision_request.max_contacts = 1000;  // 联系信息的最大值
collision_result.clear();  // 清除上次检查结果
planning_scene.checkSelfCollision(collision_request, collision_result);
ROS_INFO_STREAM("Test 5: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
collision_detection::CollisionResult::ContactMap::const_iterator it;
for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
{
  ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
}
```

#### 2.4 修改允许碰撞矩阵

`AllowedCollisionMatrix` (ACM)提供了一种机制来告诉碰撞世界忽略某些对象之间的碰撞：包括机器人的两个部分和场景中的对象。我们可以告诉碰撞检查器忽略2.3中报告的连杆之间的所有碰撞，也就是说，即使links实际上是在碰撞中，碰撞检查器也会忽略这些碰撞，并且返回not in collision。

```cpp
collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
robot_state::RobotState copied_state = planning_scene.getCurrentState();
// 碰撞结果的迭代器
collision_detection::CollisionResult::ContactMap::const_iterator it2;
for (it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); ++it2)
{
    // 忽略所有碰撞
  acm.setEntry(it2->first.first, it2->first.second, true);
}
collision_result.clear();
// 再次检查
planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);
ROS_INFO_STREAM("Test 6: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
```

#### 2.5 完整的碰撞检查（自身+环境）

我们可以使用`checkCollision`函数替代`checkSelfCollision`，它不仅检查自冲突，还能检查环境的冲突(当前为空)。这是一冲突检查函数十分常用。注意，与环境的碰撞检查将使用填充版本的机器人模型，因为填充物有助于使机器人远离环境中的障碍物

```cpp
collision_result.clear();
planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
ROS_INFO_STREAM("Test 7: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
```

### 3. 约束检查

`PlanningScene`类还包括易于使用的约束检查函数。约束可以有两种类型：(a)从运动学约束集中选择的约束:关节约束、位置约束、方位约束和可视性约束；(b)通过回调指定的用户定义约束。我们首先来看一个具有简单运动学约束的例子。

#### 3.1 检查运动学约束

首先，我们将在熊猫机器人的`panda_arm`组的末端执行器上定义一个简单的位置和方向约束。注意使用方便的函数来创建约束(这些函数可以在utils.h文件中找到，该文件在moveit_core中的kinematic_constraints目录)。

```cpp
std::string end_effector_name = joint_model_group->getLinkModelNames().back();  // 获得末端执行器的名字

geometry_msgs::PoseStamped desired_pose;
desired_pose.pose.orientation.w = 1.0;
desired_pose.pose.position.x = 0.3;
desired_pose.pose.position.y = -0.185;
desired_pose.pose.position.z = 0.5;
desired_pose.header.frame_id = "panda_link0";
moveit_msgs::Constraints goal_constraint =
    kinematic_constraints::constructGoalConstraints(end_effector_name, desired_pose);  // 创建约束的函数
```

现在，我们可以使用`PlanningScene`类中的`isStateSonstrained`函数根据这个约束检查状态。

```cpp
copied_state.setToRandomPositions();
copied_state.update();
bool constrained = planning_scene.isStateConstrained(copied_state, goal_constraint);
ROS_INFO_STREAM("Test 8: Random state is " << (constrained ? "constrained" : "not constrained"));
```

有一种更快速有效的检查约束的方法(当你想一遍又一遍地检查相同的约束时，例如在一个运动规划planner中)。我们首先构造了一个运动学约束集`KinematicConstraintSet`，该约束集对ROS约束消息进行预处理，并将其设置为快速处理。

```cpp
kinematic_constraints::KinematicConstraintSet kinematic_constraint_set(kinematic_model);
kinematic_constraint_set.add(goal_constraint, planning_scene.getTransforms());
bool constrained_2 = planning_scene.isStateConstrained(copied_state, kinematic_constraint_set);
ROS_INFO_STREAM("Test 9: Random state is " << (constrained_2 ? "constrained" : "not constrained"));
```

有一种直接的方法可以使用`KinematicConstraintSet`类来实现这一点。

```cpp
kinematic_constraints::ConstraintEvaluationResult constraint_eval_result =
    kinematic_constraint_set.decide(copied_state);
ROS_INFO_STREAM("Test 10: Random state is "
                << (constraint_eval_result.satisfied ? "constrained" : "not constrained"));
```

#### 3.2 用户定义约束

用户定义的约束也可以指定到`PlanningScene`类。这是通过使用`setStateFeasibilityPredicate`函数指定回调来实现的。下面是一个用户定义的回调函数的简单例子，它检查熊猫机器人的`panda_joint1`是处于正角还是负角。

首先根据要检查的约束条件定义检查函数。

```cpp
// 回调函数定义
bool stateFeasibilityTestExample(const robot_state::RobotState& kinematic_state, bool verbose)
{
  const double* joint_values = kinematic_state.getJointPositions("panda_joint1");
  return (joint_values[0] > 0.0);
}
```

注册该函数为回调函数，则无论何时，如果调用 `isStateFeasible`，都会触发这个用户定义的回调函数，检查是否满足条件。

```cpp
planning_scene.setStateFeasibilityPredicate(stateFeasibilityTestExample);
bool state_feasible = planning_scene.isStateFeasible(copied_state);
ROS_INFO_STREAM("Test 11: Random state is " << (state_feasible ? "feasible" : "not feasible"));
```

==注意==：如果调用`isStateValid`函数，则会执行三个检查:(a)冲突检查(b)约束检查(c)使用用户定义的回调进行可行性检查。

```cpp
bool state_valid = planning_scene.isStateValid(copied_state, kinematic_constraint_set, "panda_arm");
ROS_INFO_STREAM("Test 12: Random state is " << (state_valid ? "valid" : "not valid"));
```

注意，Moveit中所有的`planner`都会进行这三个检查。

[回到目录](#目录)

## 三、launch文件

```xml
<launch>
  <!-- send Panda urdf to parameter server -->
  <param name="robot_description" command="(findxacro)/xacro−−inorder′(find franka_description)/robots/panda_arm_hand.urdf.xacro'" />

  <include file="$(find panda_moveit_config)/launch/planning_context.launch"/>

  <node name="planning_scene_tutorial" pkg="moveit_tutorials" type="planning_scene_tutorial" respawn="false" output="screen">
    <rosparam command="load" file="$(find panda_moveit_config)/config/kinematics.yaml"/>
  </node>
</launch>
```

