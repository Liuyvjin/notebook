# RobotModel和RobotState介绍

#### 目录

[TOC]

## 一、`robot_model`和`robot_state`类简介

 `robot_model`和`robot_state`类是让您访问机器人运动学的核心类。

`RobotModel`类包含所有连杆和关节之间的关系，包括从URDF加载的关节极限属性。机器人模型还将机器人的连杆和关节分成SRDF中定义的规划组。

`RobotState`包含机器人的实时快照信息，存储关节位置的向量以及（可选的）速度和加速度，可用于获取机器人的运动学信息（这些信息取决于机器人的当前状态），比如末端执行器的雅可比矩阵。

`RobotState`还包含辅助功能，用于根据末端执行器位置(笛卡尔姿态)设置手臂位置，以及计算笛卡尔轨迹。

## 二、启动教程

```
roslaunch moveit_tutorials robot_model_and_robot_state_tutorial.launch
```

## 三、程序详解

[参考程序](<https://github.com/Liuyvjin/moveit_tutorials/blob/kinetic-devel/doc/robot_model_and_robot_state/src/robot_model_and_robot_state_tutorial.cpp>)

### 1. 创建类对象

设置开始使用`RobotModel`类非常简单。通常，您会发现大多数高级组件将返回一个指向`RobotModel`的共享指针。你应该尽可能地使用它。在本例中，我们将从这样一个共享指针开始，只讨论基本API。可以查看这些类的实际代码API，以获得关于如何使用这些类提供的更多特性的更多信息。

我们将首先实例化一个`RobotModelLoader`对象，该对象将在ROS参数服务器上查找机器人描述，并构造一个供我们使用的`RobotModel`对象的共享指针`RobotModelPtr kinematic_model`。

```cpp
robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
```

使用机器人模型`RobotModel`可以构造一个机器人状态`RobotState`（这里也是用共享指针RobotStatePtr kinematic_state）来维护机器人的配置。我们将把`RobotState`中的所有节点设置为它们的默认值。然后我们可以得到一个JointModelGroup，它代表一个特定规划组的模型，例如熊猫机器人的“panda_arm”。

> 在运动规划时，一个规划组使用move_group表示；在机器人状态中，一个规划组使用joint_model_group表示

```cpp
robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
kinematic_state->setToDefaultValues();
// 得到joint_model_group
const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
```

### 2. 获得关节值

```cpp
// 得到各个关节的名字
const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
// 得到关节值
std::vector<double> joint_values;
kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
for (std::size_t i = 0; i < joint_names.size(); ++i)
{
  ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
}
```

### 3. 关节限制

setJointGroupPositions()本身并不强制执行关节限制，但是调用enforceBounds()将执行关节限制。

```cpp
/* 将一个关节值设置在它的限制以外，使用 setJointGroupPositions() */
joint_values[0] = 5.57;
kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
/* 检查是否有关节超出了限制范围，satisfiesBounds()*/
ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
/* 强制实行设定的关节限制，再检查一遍，这次应该合法了 */
kinematic_state->enforceBounds();
ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
```

### 4. 正向运动学

现在，我们可以计算一组随机关节值的正运动学，即设定一组关节值，求末端位姿。注意，在这里我们想要找到机器人“panda_arm”组中最末端的“panda_link8”的位置。

```cpp
// 设置随机位置
kinematic_state->setToRandomPositions(joint_model_group);
// 正向运动学，获得末端的位姿
const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8");
// 输出末端位姿 
ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
```

### 5. 逆向运动学

我们现在可以求解熊猫机器人的逆运动学。为了解决IK问题，我们需要: 

* 希望求解的规划组
* 希望末端执行器达到的位姿 
* 尝试求解的次数：10次
* 每次求解的时间限制: 0.1 s

```cpp
std::size_t attempts = 10;
double timeout = 0.1;
// 使用逆向运动学的解来设置机器人状态
bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, attempts, timeout);
// 输出逆向运动学的解，就是输出每个关节的值
if (found_ik)
{
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
}
else ROS_INFO("Did not find IK solution");
```

### 6. 获得`Jacobian`矩阵

```cpp
Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
Eigen::MatrixXd jacobian;
kinematic_state->getJacobian(joint_model_group,
                             kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                             reference_point_position, jacobian);
ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
```

## 三、launch文件

```xml
<launch>
  <include file="$(find panda_moveit_config)/launch/planning_context.launch">
    <arg name="load_robot_description" value="true"/>
  </include>

  <node name="robot_model_and_robot_state_tutorial"
        pkg="moveit_tutorials"
        type="robot_model_and_robot_state_tutorial"
        respawn="false" output="screen">
    <rosparam command="load"
              file="$(find panda_moveit_config)/config/kinematics.yaml"/>
  </node>
</launch>
```

