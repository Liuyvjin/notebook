# Msg类型

#### 目录

[TOC]
## 一、std_msgs
### 1. Header
> uint32 seq
> time stamp
> string frame_id

标准数据类型，表示消息的高级的信息戳（包含编号、时间戳、坐标系）

* seq - 即sequence ID， 连续增大的消息编号 ID 
* time - 时间信息，包含两个整型的数据:
  * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
  * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    time-handling sugar is provided by the client library
* frame_id - 表示这个数据所关联的坐标系，0代表无坐标系，1代表全局坐标系

## 二、moveit_msgs

### 1. CollisionObject

定义了环境中的碰撞物体


* Header header - 信息戳
* string id - 物体的id
* object_recognition_msgs/ObjectType type - 已知对象的数据库中的对象类型？？不懂

物体的几何属性，包括相对于header.frame_id的位姿、和几何元的参数（长宽高）

* shape_msgs/SolidPrimitive[] primitives  [参考](#1. SolidPrimitive)
* geometry_msgs/Pose[] primitive_poses

面属性meshes
* shape_msgs/Mesh[] meshes
* geometry_msgs/Pose[] mesh_poses

边界平面 bounding planes (equation is specified, but the plane can be oriented using an additional pose)
* shape_msgs/Plane[] planes
* geometry_msgs/Pose[] plane_poses

操作数
* byte ADD=0 - 将对象添加到计划场景中。 如果先前存在该对象，则将其替换。

* byte REMOVE=1 - 从环境完全中删除对象（与指定的id匹配的所有内容）

* byte APPEND=2 - 表示附加到计划场景中已存在的对象上。如果对象不存在，则该物体被添加，也即变成操作0。
* byte MOVE=3 - 操作数，如果该物体已存在于场景中，该数字表示这个物体可以被移动（几何数组必须保留为空） (the geometry arrays must be left empty)
  如果需要单独移动这个物体需要设置。

* byte operation - 使用的操作数

> 使用方法：
>
> CollisionObject obj1；
>
> obj1.operation = obj1.ADD;

### 2. OrientationConstraint

这个类型定义了`姿态`约束，与此类似的还有：

```
JointConstraint.msg        PositionConstraint.msg   
OrientationConstraint.msg  VisibilityConstraint.msg 
```

信息戳

* Header header

表示约束的姿态的四元数
* geometry_msgs/Quaternion orientation

要约束的link的名字
* string link_name

可选的轴的角度的容差
float64 absolute_x_axis_tolerance
float64 absolute_y_axis_tolerance
float64 absolute_z_axis_tolerance

这个约束的权重，越接近0表示越不重要
float64 weight

### 3. Constraints

这个类型定义了一个运动规划约束的列表（它由许多2中的约束组成），一个路径必须满足所有的约束，才能被认为是有效的。
```
string name
JointConstraint[] joint_constraints
PositionConstraint[] position_constraints
OrientationConstraint[] orientation_constraints
VisibilityConstraint[] visibility_constraints
```

## 三、shape_msgs

### 1. SolidPrimitive

该类型定义了 长方体、球体、圆柱、圆锥

```shell
# 表示各种形状，这些形状的中心都定义在（0,0,0）
uint8 BOX=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 CONE=4
# 该对象的类型，取值是上述四种
uint8 type
# 该形状每个维度的取值，如长方体需要长、宽、高
float64[] dimensions
## 以下是每种形状的维度的具体索引
# box的x/y/z长度
uint8 BOX_X=0
uint8 BOX_Y=1
uint8 BOX_Z=2
# sphere的半径
uint8 SPHERE_RADIUS=0
# 对于 CYLINDER 和 CONE 类型,中线与z轴重合，圆锥尖顶朝正方向
uint8 CYLINDER_HEIGHT=0
uint8 CYLINDER_RADIUS=1
#
uint8 CONE_HEIGHT=0
uint8 CONE_RADIUS=1
```

