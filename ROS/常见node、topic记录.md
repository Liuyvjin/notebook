## 1. robot_state_publisher

这个节点监听/joint_states的消息，然后建立/tf树，再发布出去

Node [/robot_state_publisher]

1. Publications: 

 * /tf [tf2_msgs/TFMessage]  
 * /tf_static [tf2_msgs/TFMessage]  
 * /rosout [rosgraph_msgs/Log]   

2. Subscriptions: 

 * /joint_states [sensor_msgs/JointState]
```mermaid
graph LR
	a(robot_state_publisher)-->b[/tf]
	a-->c[/tf_static]
	d[/joint_states]-->a(robot_state_publisher)
```

## 2. joint_state_publisher

该节点从movegroup接收`JointState`类型的数据，并发布出来

Node [/joint_state_publisher]
Publications: 

 * /joint_states [sensor_msgs/JointState]

Subscriptions: 
 * /move_group/fake_controller_joint_states [sensor_msgs/JointState]

   这个话题好像是只有点execute以后才会开始发布。当执行结束后就停止，也就是发布jointstates的变化

```mermaid
graph LR
	a(joint_state_publisher)-->b[/joint_states]
	d[/move_group/fake_controller_joint_states]-->a(robot_state_publisher)
```

```mermaid
graph LR
	a(robot_state_publisher)-->b[/tf]
	d(joint_state_publisher)-->e[/joint_states]
	e-->a
```

