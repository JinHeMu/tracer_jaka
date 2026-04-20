# jaka_pick_place_demo

基于 **MoveIt 2** 的 `tracer_jaka_zu5` Pick &amp; Place 演示功能包（纯 RViz 可视化，使用 `mock_components`）。

本例重点在熟悉 **ROS2 下 MoveIt 的 C++ 接口**，不涉及 Gazebo、不控制夹爪、不连真实硬件。

---

## 1. 前置条件

- Ubuntu 22.04 + ROS 2 Humble（或更新）
- 已安装 MoveIt 2：`sudo apt install ros-$ROS_DISTRO-moveit`
- 已有你的 moveit_config 包（Setup Assistant 生成的那个），并且 `ros2 launch <moveit_config> demo.launch.py` 能正常启动
- 默认假设 moveit_config 包名为 `tracer_jaka_zu5_moveit_config`；如果不是，请修改：
  - `launch/pick_place_demo.launch.py` 中的 `MOVEIT_CONFIG_PKG`
  - `package.xml` 中的 `<exec_depend>` 对应项

## 2. 编译

把本功能包放到你的 ROS2 workspace（例如 `~/ros2_ws/src/`）下：

```bash
cd ~/ros2_ws
colcon build --packages-select jaka_pick_place_demo --symlink-install
source install/setup.bash
```

## 3. 运行

```bash
ros2 launch jaka_pick_place_demo pick_place_demo.launch.py
```

启动后你会看到：

1. RViz 打开，加载机械臂模型；
2. 场景里出现一张桌子和一个红色（或默认色）立方体；
3. 机械臂先回 `zero` 位，然后依次执行 pre-grasp → 下探 → attach → 抬起 → pre-place → 下放 → detach → 撤离 → 回零。

## 4. 参数调整（按实际机器人改）

在 `src/pick_place_demo.cpp` 头部：

| 参数 | 含义 | 默认 |
|---|---|---|
| `TABLE_Z` | 桌面在规划坐标系中的 z 高度 | 0.50 m |
| `CUBE_X / CUBE_Y` | 立方体位置 | 0.45, -0.15 |
| `PLACE_X / PLACE_Y` | 放置位置 | 0.45, 0.15 |
| `APPROACH_OFFSET` | 预接近距离 | 0.10 m |

如果规划参考系不是 `base_link`（节点启动时日志会打印），桌子和立方体的 x/y/z 需要基于那个坐标系来设置。

## 5. ROS1 → ROS2 速查

| 场景 | ROS1 | ROS2 |
|---|---|---|
| 节点 | `ros::NodeHandle nh;` | `auto node = rclcpp::Node::make_shared("...");` |
| 异步 spinner | `ros::AsyncSpinner s(1); s.start();` | `rclcpp::executors::SingleThreadedExecutor; std::thread spin` |
| move_group 构造 | `MoveGroupInterface mg("jaka_arm");` | `MoveGroupInterface mg(node, "jaka_arm");` |
| 消息类型 | `moveit_msgs::CollisionObject` | `moveit_msgs::msg::CollisionObject` |
| 日志 | `ROS_INFO("...")` | `RCLCPP_INFO(logger, "...")` |
| 规划返回值 | `MoveItErrorCode::SUCCESS` | `moveit::core::MoveItErrorCode::SUCCESS` |
| 四元数转换 | `tf2_geometry_msgs.h` | `tf2_geometry_msgs.hpp`（多一个 `p`）|
| launch 文件 | `.launch` XML | `.launch.py`（Python） |
| 参数注入 | `rosparam load` | `MoveItConfigsBuilder(...).to_moveit_configs()` |

## 6. 下一步建议

- 熟悉后可以把 `attach/detach` 替换成对真实夹爪话题的发布（如果走 JointTrajectory 直驱）；
- 若需要用 **MoveIt Task Constructor (MTC)** 做更完善的 pick/place，可以在此基础上扩展；
- 真机时把 `mock_components/GenericSystem` 换成你的 JAKA 硬件插件即可，演示代码无需改动。
