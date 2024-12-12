# 考核规则及评分细则

### 必做

**总分: 35**
如果没有完成必做需求，没有资格参加答辩。

#### 1. 运行 [rm_description](https://github.com/YoujianWu/rm_description_for_task.git) 的英雄机器人模型

**分数: 5**

参考资料：

- [rm_description](https://github.com/YoujianWu/rm_description_for_task.git)

#### 2. 学习理解并运行 [simple_chassis_controller](https://github.com/YoujianWu/simple_chassis_controller)

**分数: 10**

要求：

1. 全程使用 CLion 进行编译和 DeBug （可在群里寻求帮助）

2. 如需使用catkin命令行工具，使用 `catkin-tool` 而不是 `catkin_make` (可在群里寻求帮助)

3. 理解 `SimpleChassisController` 的类成员意义

   类成员：front_left_joint\_,  front_right_joint\_, back_left_joint\_, back_right_joint\_, tau, cmd\_, state\_, last_change\_

   - front_left_joint\_,  front_right_joint\_, back_left_joint\_, back_right_joint\_：四个轮子的句柄，使用类函数 setCommand() 可以发送控制命令
   - tau：用来计算力矩
   - cmd\_：代表每个状态对应发布的指令
   - state_: 代表控制器不同的状态，每个状态对应一个行为
   - last_change_: 上一次状态改变的时间

   

4. 理解控制器怎么被加载进模拟器和实际机器人中 (pluginlib)

   控制器之类的插件，通过pluginlib动态链接并加载进ros系统，为我们提供控制机器人的硬件接口。

   （pluginlib是一个c++库）

   基于ControllerBase这个基类，自己编写一个衍生类（控制器插件）SimpleChassisController，将插件注册(PLUGINLIB_EXPORT_CLASS)、构建插件库(add_library)和链接到ros工具链(.xml)后，就可以用pluginlib这个c++库来导入和使用插件了。在simple_chassis_controller包中是直接将控制器插件加载进controller_spawner这个节点里，来管理硬件接口。

参考资料：

- [CLion ROS setup tutorial](https://www.jetbrains.com/help/clion/ros-setup-tutorial.html)
- [CLion Attach to process](https://www.jetbrains.com/help/clion/attaching-to-local-process.html)
  调试simple_chassis_controller时, 应该 attach 到 `gzsever`
- [simple_chassis_controller]
- [ros_control: A generic and simple control framework for ROS](http://www.theoj.org/joss-papers/joss.00456/10.21105.joss.00456.pdf)
- [ros_control: wiki](https://github.com/ros-controls/ros_control/wiki)
- [Gazbeo: Tutorial: ROS Control](http://gazebosim.org/tutorials/?tut=ros_control)
- [pluginlib](http://wiki.ros.org/pluginlib)

#### 3. 谈谈你对 ros_control 的理解

**分数: 20**

ros_control就是一个框架，用来更加方便地对机器人进行控制。

包括以下几个部分：controller_manager（控制器管理器），controller_interface（控制器接口），hardware_interface（硬件接口），transmission_interface（传动接口）。

ros_control大概的工作流程就是：控制器管理器加载不同的控制器到控制器接口，随后控制器接口对硬件接口进行操作。硬件接口得到命令后对具体的硬件进行操作（write()），随后读取（read()）硬件状态返回给控制器，更新控制器命令。

参考资料：

- [ros_control: A generic and simple control framework for ROS](http://www.theoj.org/joss-papers/joss.00456/10.21105.joss.00456.pdf)
- [ros_control: wiki](https://github.com/ros-controls/ros_control/wiki)

### 选做

**总分: 115** 尽量完成更多项

#### 1. 创建新的 ros_package

**分数: 10**

要求：

1. 以 rm_template 为模版 创建新仓库, 名为 hero_chassis_controller
2. 正确填写 **CMakeList.txt**, **package.xml**, **README.md**, 在完成后面的需求时记更新

参考资料：

- [rm_template](https://github.com/gdut-dynamic-x/rm_template)
- [catkin/CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt)
- [package.xml](http://wiki.ros.org/catkin/package.xml)

#### 2. 完善的版本管理

**分数： 10**

要求：

1. 合理的commit
2. 清晰的commit消息
3. 有和合并冲突的处理
4. 分工明确（组内贡献度将参考版本管理）

#### 3. 正确的代码规范

**分数： 10**

要求：

1. 使用 ROS C++ Style Guide 或者 Google C++ Style.
2. 将本项目的 .clang-format 与 .clang-tidy
   文件添加到你的项目中，并使用 [Save Actions](https://plugins.jetbrains.com/plugin/7642-save-actions) 来优化代码格式

参考资料：

- [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)
- [Google C++ Style](https://google.github.io/styleguide/cppguide.html)

#### 4. 使用 PID 控制轮子的速度

**分数： 20**

要求：

1. 一个PID对应控制一个轮子（共四个PID）
2. 使用 control_toolbox 的 PID 控制器
3. 可在配置文件中设定 PID 的各个参数
4. 会使用 plotjuggler 和 rqt 的 rqt_reconfigure 调整得到合适的PID参数
5. 理解 `control_toolbox::Pid` 类 参考资料：

- [control_toolbox](http://wiki.ros.org/control_toolbox)
- [control_toolbox::Pid Class Reference](http://docs.ros.org/en/jade/api/control_toolbox/html/classcontrol__toolbox_1_1Pid.html)
- [effort_controllers/joint_velocity_controller.cpp](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/effort_controllers/src/joint_velocity_controller.cpp)
  展示了如何用 `control_toolbox::Pid` 控制单个关节的速度
- [plotjuggler](https://www.ros.org/news/2017/01/new-package-plotjuggler.html)
- [rqt_reconfigure](http://wiki.ros.org/rqt_reconfigure)

#### 5. 使用逆运动学计算各个轮子的期望速度

**分数： 20**

要求：

可以底盘自旋

1. 底盘坐标系定义参照仿真中的 base_link
2. 接收 "/cmd_vel" 上的 [geometry_msgs/Twist](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Twist.html)
   数据，将它视为底盘坐标系下的速度指令，计算各个轮子期望的转速
3. 可在配置文件中设定 底盘的轴距 轮距

参考资料：

- [geometry_msgs/Twist](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Twist.html)
- [Mecanum wheel](https://en.wikipedia.org/wiki/Mecanum_wheel)
- [Kinematic Model of a Four Mecanum Wheeled Mobile Robot](https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf)
  这篇论文真真真水...

#### 6. 使用正运动学实现里程计

**分数： 15**

要求：

1. 根据轮子的实际速度计算底盘的速度，并进行叠加实现里程计
2. 将数据发以 [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) 发布到 topic "/odom" 上
3. 将数据处理为坐标变换关系： **"odom"** 到 **"base_link"**

参考资料：

- [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)
- [navigationTutorialsRobotSetupOdom](http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom)
- [Kinematic Model of a Four Mecanum Wheeled Mobile Robot](https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf)

#### 7. 使用tf计算实现世界坐标下的速度控制

移动机器人的里程计就是**机器人每时每刻在世界坐标系下位姿状态**

**分数： 15**

要求：

在配置文件中设定速度模式参数vel_com="map"or"odom",

1. 已实现 `5. 使用逆运动学计算各个轮子的期望速度` 和 `6. 使用正运动学实现里程计`
2. 将 topic "/cmd_vel" 中的速度指令视为世界坐标系下(**"odom"** 或者 **"map"**)， 通过 `tf` 变换到 底盘坐标系后执行。
3. 可在配置文件中设定 **“底盘坐标系速度模式”** ( 即`5. 使用逆运动学计算各个轮子的期望速度`) 和 **“全局坐标系速度模式”** 两个模式

参考资料：

- [tfOverviewUsing Published Transforms](http://wiki.ros.org/tf/Overview/Using%20Published%20Transforms
  ) 使用 `transformVector`

#### 8. 其他特色功能

**分数： 15**

要求：

1. 自由发挥
2. 如 使用 teleop_twist_keyboard 键盘操控底盘
3. 如 可在配置文件设定底盘加速度。
4. 如 可在配置文件设定底盘的运动功率，编写功率约束算法，并保证运动时不超出预期功率。

参考资料：

- [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard)
- [power_limit](https://hz-rm-bbs-web-prod.oss-cn-hangzhou.aliyuncs.com/attachment/pre/v1/202309/15/000721u4euqvq7vsijvgpk.attach/%E5%B9%BF%E4%B8%9C%E5%B7%A5%E4%B8%9A%E5%A4%A7%E5%AD%A6%E4%B8%8B%E4%BE%9B%E5%BC%B9%E6%8A%80%E6%9C%AF%E6%96%B9%E6%A1%88.pdf)

[rm_description]: https://github.com/gdut-dynamic-x/rm_description

[simple_chassis_controller]: https://github.com/gdut-dynamic-x/simple_chassis_controller

[catkin-tool]: https://catkin-tools.readthedocs.io/
