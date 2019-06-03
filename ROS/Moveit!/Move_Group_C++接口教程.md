# Move Group C++接口教程

#### 目录

[TOC]

## 一、启动程序

使用RViz进行可视化：

```shell
roslaunch panda_moveit_config demo.launch
```

启动教程程序：

```shell
roslaunch moveit_tutorials move_group_interface_tutorial.launch
```

注意打开RViz的**RvizVisualToolsGui**面板来一步步运行程序。

## 二、程序详解

[程序链接](<https://github.com/Liuyvjin/moveit_tutorials/blob/kinetic-devel/doc/move_group_interface/src/move_group_interface_tutorial.cpp>)

### 1. 头文件

```cpp
// move_group 接口
#include <moveit/move_group_interface/move_group_interface.h>
// 添加场景物体
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// moveit_msgs
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
// 可视化工具
#include <moveit_visual_tools/moveit_visual_tools.h>
```

### 2. 初始化ROS节点

```cpp
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ...
}
```

### 3. 创建类对象

`Moveit!`的操作对象是规划组，它用`JointModelGroup`对象保存。

在整个过程中，`planning group`和`joint model group`这两个术语可以互换使用。

```cpp
// 设置规划组名称
static const std::string PLANNING_GROUP = "panda_arm";
// 使用规划组名称，创建 MoveGroup 类对象
moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
```

 `Moveit`的`PlanningSceneInterface`可以用来添加和移除碰撞物体，我们创建一个类对象备用。

```cpp
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
```

为了提高性能，经常使用原始指针`robot_state::JointModelGroup*`来引用规划组，它和`move_group`指的是同一个东西。

```cpp
const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
```

>```cpp
>class MoveGroup{
>    // 获得机器人状态的指针
>	robot_state::RobotStatePtr getCurrentState();
>}
>class RobotState{
>	// 获得一个规划组的关节模型指针
>        const JointModelGroup* getJointModelGroup(const std::string& group) const
>      {
>        return robot_model_->getJointModelGroup(group);
>      }
>}
>
>RobotModelConstPtr robot_model_;
>robotmodel 和 robotstate到底什么关系？？？
>```
>

### 4. 可视化工具

`MoveItVisualTools`类提供了许多使用rviz可视化的工具，包括物体、机器人、轨迹、甚至可以一步一步运行脚本来可视化。

```cpp
namespace rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
visual_tools.deleteAllMarkers();
```

`Remote control`是一种检查工具，允许用户通过`RViz`中的按钮和键盘快捷键逐步执行高级脚本。

```cpp
visual_tools.loadRemoteControl();
```

RViz提供了许多类型的标记，在这个演示中，我们将使用文本、柱面和球体。

```cpp
Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
text_pose.translation().z() = 1.75;
visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
```

批量发布用于减少发送到RViz的用于大型可视化的消息数量，使用这个函数触发一次发布。

```cpp
visual_tools.trigger();
```

暂停程序，直到用户按下`RvizVisualToolsGui`窗口中的`next`

```cpp
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
```

### 5. 规划`Pose`目标

```cpp
// 创建一个Pose作为目标
geometry_msgs::Pose target_pose1;
target_pose1.orientation.w = 1.0;
target_pose1.position.x = 0.28;
target_pose1.position.y = -0.2;
target_pose1.position.z = 0.5;
// 将pose设置为move_group的目标
move_group.setPoseTarget(target_pose1);
// 保存一个 plan 的对象
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
// 规划改目标，返回成功或失败
bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
// 输出提示信息
ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
```

### 6. 可视化Plans

使用可视化工具展示更多信息

```cpp
ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
visual_tools.publishAxisLabeled(target_pose1, "pose1");
visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
visual_tools.trigger();
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
```

### 7. 移动到目标Pose

也即执行刚才做的规划。注意，我们之前设定的姿势目标仍然是活动的，所以机器人会尝试移动到那个目标。在本教程中，我们将不使用该函数，因为它是一个阻塞函数，需要控制器处于活动状态，并报告轨迹执行成功。

```cpp
/* Uncomment below line when working with a real robot */
/* move_group.move(); */
```

### 8. 规划`joint-space`目标

刚刚是用末端的`Pose`表示一个状态，但其实还可一用关节值来表示一个状态，规划组中所有关节的取值范围组成了一个关节空间。接下来就用关节值作为目标状态进行规划。

首先，定义一个指向当前`RobotState`的的指针，`RobotState`是一个类，包含了当前所有位置/速度/加速度的数据。

```cpp
moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
```

获得当前的这个规划组（`joint_model_group`）的所有关节值：

```cpp
std::vector<double> joint_group_positions;
current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
```

根据前关节值，修改一下作为目标状态；并设置为`move_group`的目标关节值；规划并可视化

```cpp
joint_group_positions[0] = -1.0;  // radians
move_group.setJointValueTarget(joint_group_positions);  // 设置目标关节值 
// 规划
success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
// 可视化
visual_tools.deleteAllMarkers();
visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
visual_tools.trigger();
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
```

### 9. 路径约束下规划路径

路径约束是约束某一个`link`的姿态，在本教程中约束的是`panda_link7`

定义路径约束

```cpp
moveit_msgs::OrientationConstraint ocm;
ocm.link_name = "panda_link7";  // 约束的link
ocm.header.frame_id = "panda_link0";  // 参考系
ocm.orientation.w = 1.0;  // 四元数姿态
ocm.absolute_x_axis_tolerance = 0.1;  // x轴角度容差
ocm.absolute_y_axis_tolerance = 0.1;
ocm.absolute_z_axis_tolerance = 0.1;
ocm.weight = 1.0;  // 权重
```

为规划组设置路径约束

```cpp
moveit_msgs::Constraints test_constraints;  // 约束列表
test_constraints.orientation_constraints.push_back(ocm);  
move_group.setPathConstraints(test_constraints);  // 设置为规划组的约束
```

由于要保证起点位置也符合路径约束，所以要先设置规划的起点：
由于设置起点位置的`setStartStare`函数的原型为:

> void setStartState(const robot_state::RobotState& start_state);

所要先构造一个`RobotState`类型的数据，可以用当前的状态来初始化。
```cpp
robot_state::RobotState start_state(*move_group.getCurrentState()); 
geometry_msgs::Pose start_pose2;
start_pose2.orientation.w = 1.0;
start_pose2.position.x = 0.55;
start_pose2.position.y = -0.05;
start_pose2.position.z = 0.8;
start_state.setFromIK(joint_model_group, start_pose2);
move_group.setStartState(start_state);
```

现在可以开始规划了，目标状态不做改变，依然使用之前的。注意使用`setPlanningTime()`函数增加规划时间，因为有约束的问题较为复杂。

```cpp
move_group.setPlanningTime(10.0);
success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");
// 可视化
visual_tools.deleteAllMarkers();
visual_tools.publishAxisLabeled(start_pose2, "start");
visual_tools.publishAxisLabeled(target_pose1, "goal");
visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
visual_tools.trigger();
visual_tools.prompt("next step");
```

当规划好之后，**注意**要清除约束，否则约束一直存在。

```cpp
move_group.clearPathConstraints();
```

现在起点状态没有约束要求了，因此可以设置回当前状态。

```cpp
move_group.setStartStateToCurrentState();
```

### 10. 笛卡尔路径规划

通过输入末端的离散位姿构成一条空间轨迹。在moveit中提供`computeCartesianPath`函数用于路径规划，该函数为直线插补。由于我们之前设定了起始状态，所以在离散的`Pose`序列`waypoints list`中不用设置起始点，但是添加起始点有助于可视化。

构造Pose的列表

```cpp
// 获得当前位姿作为初始Pose
geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;
// waypoints list
std::vector<geometry_msgs::Pose> waypoints; 
waypoints.push_back(target_pose3);
// 向下
target_pose3.position.z -= 0.2;
waypoints.push_back(target_pose3);  
// 向右
target_pose3.position.y -= 0.2;
waypoints.push_back(target_pose3);  
// up and left
target_pose3.position.z += 0.2;
target_pose3.position.y += 0.2;
target_pose3.position.x -= 0.2;
waypoints.push_back(target_pose3);  
```

笛卡尔路径运动经常要求某些运动有较慢的速度，如接近和后退抓取运动。在这里，我们演示了如何通过对每个关节的最大速度乘以比例因子来降低机器人手臂的速度。注意这不是末端执行器dui的速度。

```cpp
move_group.setMaxVelocityScalingFactor(0.1);
```

我们想要笛卡尔坐标路径以1cm的分辨率插值，所以指定0.01作为平移的最大步长。`jump threshold`是机械臂最小移动距离，设置0.0则忽略检查。（==警告==在操作真正的硬件时禁用跳转阈值可能会导致冗余关节的不可预测的大运动，并可能是一个安全问题。）

```cpp
moveit_msgs::RobotTrajectory trajectory;  // 计算得到的路径保存在该数据中
const double jump_threshold = 0.0;
const double eef_step = 0.01;
// 计算笛卡尔路径
double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
```

可视化

```cpp
visual_tools.deleteAllMarkers();
visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
for (std::size_t i = 0; i < waypoints.size(); ++i)
  visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
visual_tools.trigger();
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

```

### 11. 向场景中添加/删除/附加/分离物体

定义一个物体，并添加到场景中，进行规划

```cpp
moveit_msgs::CollisionObject collision_object;
collision_object.header.frame_id = move_group.getPlanningFrame();
collision_object.id = "box1";
// 形状
shape_msgs::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);
primitive.dimensions[0] = 0.4;
primitive.dimensions[1] = 0.1;
primitive.dimensions[2] = 0.4;
// 位置
geometry_msgs::Pose box_pose;
box_pose.orientation.w = 1.0;
box_pose.position.x = 0.4;
box_pose.position.y = -0.2;
box_pose.position.z = 1.0;
collision_object.primitives.push_back(primitive);
collision_object.primitive_poses.push_back(box_pose);
collision_object.operation = collision_object.ADD;  // 对该物体执行添加操作
// 加入物体列表
std::vector<moveit_msgs::CollisionObject> collision_objects;
collision_objects.push_back(collision_object);
// 将物体列表加入场景
ROS_INFO_NAMED("tutorial", "Add an object into the world");
planning_scene_interface.addCollisionObjects(collision_objects);
// 可视化
visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
visual_tools.trigger();
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");
// 接下来的运动规划会自动避障
move_group.setStartState(*move_group.getCurrentState());
geometry_msgs::Pose another_pose;
another_pose.orientation.w = 1.0;
another_pose.position.x = 0.4;
another_pose.position.y = -0.4;
another_pose.position.z = 0.9;
move_group.setPoseTarget(another_pose);

success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");
// 可视化
visual_tools.deleteAllMarkers();
visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
visual_tools.trigger();
visual_tools.prompt("next step");
```

附加和分离物体

```cpp
// 附加
ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
move_group.attachObject(collision_object.id);
// 可视化
visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
visual_tools.trigger();
// 等待物体附加上
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "
                    "robot");
// 分离
ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
move_group.detachObject(collision_object.id);
// 可视化
visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
visual_tools.trigger();
// 等待物体分离
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the "
                    "robot");
```

移除物体

```cpp
// 移除
ROS_INFO_NAMED("tutorial", "Remove the object from the world");
std::vector<std::string> object_ids;
object_ids.push_back(collision_object.id);
planning_scene_interface.removeCollisionObjects(object_ids);
// 可视化
visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
visual_tools.trigger();
// 等待物体被移除
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");
```

