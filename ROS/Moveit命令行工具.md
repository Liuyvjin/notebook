# Moveit 命令行工具

#### 目录

[TOC]

## 启动

```shell
 rosrun moveit_commander moveit_commander_cmdline.py
```

## 命令

Known commands:
  help                显示帮助信息
  allow looking <true|false>       enable/disable looking around
  allow replanning <true|false>    enable/disable replanning
  constrain           清除路径约束
  constrain <name>    使用名为<name>的路径约束
  current             输出规划组的当前状态
  database            展示当前连接的数据库（如果有的话）
  delete <name>       删除名为<name>的joint的值
  eef                 输出当前规划组的末端执行器名称
  execute             执行一个已经计算好的plan

> 一般使用：
>
> plan goal
>
> execute

  go <name>           规划并执行名为<name>的状态
  go rand             规划并执行一个随机状态
  go <dir> <dx>|     使末端朝一个指定的方向运动dx，<dir>可选的值有up|down|left|right|forward|backward 

  ground              为规划场景添加一个地面
  id|which            输出当前规划组的名字
  joints              显示当前规划组中joints的名字
  load <file>         载入一个脚本file，这个脚本必须放在当前文件夹下
  pick <name>         拿起object <name>
  place <name>        放下object <name>
  plan <name>         规划一个到达状态<name>的运动
  plan rand           规划一个到达随机状态的运动
  planner <name>      使用planner<name>来规划下一运动
  record <name>       保存当前关节值在<name>中,可简写为rec

> 例如：
>
> rec c
>
> c[0] = 1.0  # 改变第一个关节值
>
> g = c  # 将关节值保存在另一个变量中
>
> b = [1 1 1 1 1 1]  # 定义一个变量，一定要有中括号，数字间不能有逗号
>
> c = b  # 赋值给c

  rotate <x> <y> <z>  分别绕x，y，z旋转
  save [<file>]       保存当前所有变量为一个文件，下次load就行了
  show                输出所有变量
  show <name>         输出指定变量
  stop                stop the active group
  time                输出设置的一次规划的限制时间
  time <val>          设置一次规划的限制时间
  tolerance           输出达到目标的容许的误差
  tolerance <val>     设置容差
  trace <on|off>      enable/disable replanning or looking around
  use <name>          使用名为<name>的规划组 (如果是第一次使用，会自动加载)
  use|groups          展示所有已经被加载进来的规划组
  vars                输出所有的变量的名字
  wait <dt>           睡眠等待 <dt> 秒
  x = y               把y的值赋给x
  x = [v1 v2...]      为x赋值
  x[idx] = val        为x的一个元素赋值

