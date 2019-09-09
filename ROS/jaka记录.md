# 1. js_pub

启动节点：j_state_pub

作用：上传数据。与机器人连接，读取机器人的joint_states，并发布。

发布话题：

* /joint_states  (sensor_msgs::JointState)
* /feedback_states (control_msgs::FollowJointTrajectoryFeedback)

# 2. joint_state_pub

启动节点： joint_states_pub

作用：下发数据。与机器人连接，将移动指令发给机器人，使机器人移动。

订阅话题：/joint_path_command

# 3. moveit_planning.launch

包含：

+ move_group.launch
+ planning_context.launch