# PlanningScene ROS API 教程

#### 目录

[TOC]

## 一、教程简介

在本教程中，我们将研究如何使用 `Planning Scene` `diffs`来执行两个操作:

* 向世界添加和删除物体
* 在机器人上附加和分离物体

## 二、运行程序

```shell
roslaunch panda_moveit_config demo.launch
roslaunch moveit_tutorials planning_scene_ros_api_tutorial.launch
```

其中第二个launch文件：

```xml
<launch>
  <node name="planning_scene_ros_api_tutorial" pkg="moveit_tutorials" type="planning_scene_ros_api_tutorial" respawn="false" output="screen">
  </node>
</launch>
```



## 三、程序详解

[参考程序](<https://github.com/Liuyvjin/moveit_tutorials/blob/kinetic-devel/doc/planning_scene_ros_api/src/planning_scene_ros_api_tutorial.cpp>)

### 1. ROS API 

规划场景（Planning Scene）发布者的`ROS API`是通过使用`diffs`的话题（topic）实现的。规划场景差异(`diff`)是当前规划场景(由`move_group`节点维护)和用户希望的新规划场景之间的差异。

### 2. 发布所需的话题

我们创建一个发布者并等待订阅者。注意，这个主题可能需要在`launch`文件中重新映射。

```cpp
ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
ros::WallDuration sleep_t(0.5);  //  等待对象
while (planning_scene_diff_publisher.getNumSubscribers() < 1)
{
  sleep_t.sleep();  // 当没有订阅者时，睡眠一段时间
}
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
```

### 3. 定义附加物体消息

我们将使用此消息从世界中添加或删除该物体对象。

> moveit_msgs::AttachedCollisionObject ：用于描述 robot_state 中附加分离物体的操作 
>
> moveit_msgs::CollisionObject ：用于描述 world 中添加移除物体的操作 
>
> 前者是对于后者的进一步封装，AttachedCollisionObject.object 其实就是 CollisionObject 类型的。

```cpp
moveit_msgs::AttachedCollisionObject attached_object;
/* 附加到哪个link上 */
attached_object.link_name = "panda_leftfinger";
/* 参考坐标系 */
attached_object.object.header.frame_id = "panda_leftfinger";
/* 物体的id */
attached_object.object.id = "box";
/* 设置物体的位姿 */
geometry_msgs::Pose pose;
pose.orientation.w = 1.0;
/* 设置物体的形状 */
shape_msgs::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);
primitive.dimensions[0] = 0.1;
primitive.dimensions[1] = 0.1;
primitive.dimensions[2] = 0.1;
/* 将位姿和形状赋予物体 */
attached_object.object.primitives.push_back(primitive);
attached_object.object.primitive_poses.push_back(pose);
```

将物体的操作定义为`ADD`，意为将物体添加到`world`中的操作

```cpp
attached_object.object.operation = attached_object.object.ADD;
```

由于我们将对象附加到机械手上以模拟拾起对象，所以我们希望碰撞检查器忽略对象和机械手之间的碰撞。只需要定义默认与物体接触的link即可：

```cpp
attached_object.touch_links = std::vector<std::string>{ "panda_hand", "panda_leftfinger", "panda_rightfinger" };
```

### 4. 向环境中添加物体 （发布`diff`消息）

将对象添加到环境中，方法是将其添加到规划场景的`world`部分的碰撞对象集合中。注意，这里我们只使用`attached_object`消息的`object`字段。

```cpp
ROS_INFO("Adding the object into the world at the location of the hand.");
moveit_msgs::PlanningScene planning_scene;
// 将物体添加到规划场景的world字段中
planning_scene.world.collision_objects.push_back(attached_object.object);
// 这个规划场景表示的是 diff
planning_scene.is_diff = true;
// 发布这个消息
planning_scene_diff_publisher.publish(planning_scene);
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
```

### 5. 插曲: 同步与异步更新

有两种单独的机制可以使用`diffs`与`move_group`节点交互：

+ 通过`rosservice call`发送`diff`并阻塞，直到应用了`diff`(同步更新)
+ 通过`topic`发送`diff`，程序不阻塞，即使`diff`可能还没有应用(异步更新)

虽然本教程的大部分内容使用后一种机制(考虑到可以在等待可视化的时候插入长睡眠，异步更新不会造成问题)，但是完全有理由使用以下服务客户机替换`planning_scene_diff_publisher`：

```cpp
ros::ServiceClient planning_scene_diff_client =
    node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
planning_scene_diff_client.waitForExistence();
```

并通过服务调用将差异发送到规划场景：

```cpp
moveit_msgs::ApplyPlanningScene srv;  // 服务请求消息
srv.request.scene = planning_scene;
planning_scene_diff_client.call(srv);  // 发送服务请求
```

### 6. 附加物体到机器人上

当机器人从环境中捡起一个物体时，我们需要将该物体“附加”到机器人上，这样任何与机器人模型相关的组件都知道考虑该附加物体，例如碰撞检查。

**附加对象需要对`diff`消息做两个操作**：

+ 从`world`字段中清除添加物体操作，新增删除物体操作
+ 将物体附加到`robot_state`字段上

```cpp
/* 首先定义 REMOVE object message */
moveit_msgs::CollisionObject remove_object;
remove_object.id = "box";
remove_object.header.frame_id = "panda_link0";
remove_object.operation = remove_object.REMOVE;
```

注意，我们如何确保diff消息不包含其他操作，方法是首先清除这些字段

```cpp
/* 执行 REMOVE + ATTACH 操作 */
ROS_INFO("Attaching the object to the hand and removing it from the world.");

/* world 字段修改 */
planning_scene.world.collision_objects.clear(); // 清除之前添加的添加物体操作
planning_scene.world.collision_objects.push_back(remove_object);  // 将删除物体操作写入

/* robot_state 字段修改 */ 
planning_scene.robot_state.attached_collision_objects.push_back(attached_object);  // 添加附加物体操作

/* 发布 diff 消息 */
planning_scene_diff_publisher.publish(planning_scene);
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
```

### 7. 从机器人上分离物体

从机器人上分离物体需要两个操作：

+ 将物体与机器人分离（修改`robot_state`）
+ 将物体重新添加到世界中（修改`world`）

```cpp
/* 首先定义 分离物体操作 */
moveit_msgs::AttachedCollisionObject detach_object;
detach_object.object.id = "box";
detach_object.link_name = "panda_link8";
detach_object.object.operation = attached_object.object.REMOVE;
```

注意，我们如何确保`diff`消息不包含其他操作（之前定义的操作），方法是首先清除这些字段。

```cpp
/* 执行 DETACH + ADD 操作 */
ROS_INFO("Detaching the object from the robot and returning it to the world.");
// 清除之前对 robot_state 的操作
planning_scene.robot_state.attached_collision_objects.clear();
// 新增分离操作
planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
// 这个字段是 diff
planning_scene.robot_state.is_diff = true;
// 清除之前对 world 的操作
planning_scene.world.collision_objects.clear();
// 新增添加操作
planning_scene.world.collision_objects.push_back(attached_object.object);
// 这个字段是 diff
planning_scene.is_diff = true;
// 发布 diff
planning_scene_diff_publisher.publish(planning_scene);
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
```

### 8. 从世界中移除对象

操作与之前类似

```cpp
ROS_INFO("Removing the object from the world.");
// 先清除 diff 中的操作
planning_scene.robot_state.attached_collision_objects.clear(); 
planning_scene.world.collision_objects.clear(); 
// 添加删除物体操作+发布
planning_scene.world.collision_objects.push_back(remove_object);
planning_scene_diff_publisher.publish(planning_scene);
```

[回到目录](#目录)



