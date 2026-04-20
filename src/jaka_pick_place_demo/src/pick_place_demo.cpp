// =============================================================================
//  jaka_pick_place_demo — MoveIt 2 Pick & Place 教学示例
//  目标：帮助熟悉 ROS1 MoveIt 的用户快速上手 ROS2 的接口
//
//  与 ROS1 的主要差异（重点看注释标注的 [ROS2]）：
//   1. Node：rclcpp::Node::SharedPtr 代替 ros::NodeHandle
//   2. 必须用 executor spin 一个节点，MoveGroupInterface 内部要靠它收 TF/状态
//   3. MoveGroupInterface 构造：(node, group_name)，不再是只传 group_name
//   4. 消息头命名空间：xxx_msgs::msg::Yyy（多一层 ::msg::）
//   5. 日志宏：RCLCPP_INFO(logger, ...)，logger 从 node 拿
//   6. launch 用 Python，通过 MoveItConfigsBuilder 统一注入参数
// =============================================================================

#include <memory>
#include <thread>
#include <chrono>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

// ----------- 常量配置（根据需要调整） -----------
static const std::string PLANNING_GROUP   = "jaka_arm";   // 来自 SRDF 的 <group name="jaka_arm">
static const std::string TARGET_CUBE_ID   = "target_cube";
static const std::string TABLE_ID         = "table";
static const std::string PLACE_ZONE_ID    = "place_zone"; // 视觉参考，非必须

// 默认规划参考系在 URDF 根链接（一般是 base_link），如果你的规划坐标系不同，
// 代码中会用 move_group_.getPlanningFrame() 自动获取。
// 下面这些坐标是 "世界/基座" 下的位置，按你的机械臂实际安装高度调整。
static constexpr double TABLE_X       = 0.45;
static constexpr double TABLE_Y       = 0.00;
static constexpr double TABLE_Z       = 0.50;   // 机械臂基座离桌面的高度差（假设 tracer 约 0.5 m）
static constexpr double TABLE_SX      = 0.60;
static constexpr double TABLE_SY      = 0.60;
static constexpr double TABLE_SZ      = 0.02;

static constexpr double CUBE_SIZE     = 0.04;
static constexpr double CUBE_X        = 0.45;
static constexpr double CUBE_Y        = -0.15;

static constexpr double PLACE_X       = 0.45;
static constexpr double PLACE_Y       = 0.15;

// 抓取预备位姿相对立方体的偏移
static constexpr double APPROACH_OFFSET = 0.10;  // 预抓取在物体上方 10 cm
static constexpr double GRASP_OFFSET    = 0.02;  // 最终 TCP 位置偏移

// =============================================================================
class PickPlaceDemo
{
public:
  explicit PickPlaceDemo(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    logger_(node->get_logger()),
    // [ROS2] 构造函数第一个参数是 node
    move_group_(node, PLANNING_GROUP)
  {
    // 规划参数 —— 对应 ROS1 的 setPlanningTime / setNumPlanningAttempts 等
    move_group_.setPlanningTime(10.0);
    move_group_.setNumPlanningAttempts(10);
    move_group_.setMaxVelocityScalingFactor(0.3);
    move_group_.setMaxAccelerationScalingFactor(0.3);
    move_group_.setGoalPositionTolerance(0.005);
    move_group_.setGoalOrientationTolerance(0.01);

    RCLCPP_INFO(logger_, "Planning frame     : %s", move_group_.getPlanningFrame().c_str());
    RCLCPP_INFO(logger_, "End effector link  : %s", move_group_.getEndEffectorLink().c_str());
    RCLCPP_INFO(logger_, "Planning group     : %s", PLANNING_GROUP.c_str());
  }

  void run()
  {
    // 1. 场景里放一张桌子和一个目标立方体
    addCollisionObjects();
    rclcpp::sleep_for(1s);

    // 2. 回零位
    goToNamedTarget("zero");

    // 3. 抓取
    pick();

    // 4. 放置
    place();

    // 5. 再次回零位
    goToNamedTarget("zero");

    RCLCPP_INFO(logger_, "Pick & place demo finished ✓");
  }

private:
  // ---------------------------------------------------------------------------
  // 规划到 SRDF 里定义的命名位姿（例如 "zero"）
  // 对应 ROS1：move_group.setNamedTarget("zero"); move_group.move();
  // ---------------------------------------------------------------------------
  bool goToNamedTarget(const std::string & name)
  {
    RCLCPP_INFO(logger_, "--> Moving to named target: %s", name.c_str());
    move_group_.setNamedTarget(name);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto ok = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) {
      RCLCPP_ERROR(logger_, "Planning to %s FAILED", name.c_str());
      return false;
    }
    return move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  // ---------------------------------------------------------------------------
  // 规划到某个末端位姿
  // ---------------------------------------------------------------------------
  bool goToPose(const geometry_msgs::msg::Pose & target, const std::string & label)
  {
    RCLCPP_INFO(logger_, "--> Moving to pose: %s  (x=%.3f y=%.3f z=%.3f)",
                label.c_str(), target.position.x, target.position.y, target.position.z);

    move_group_.setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto ok = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) {
      RCLCPP_ERROR(logger_, "Planning to %s FAILED", label.c_str());
      return false;
    }
    return move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  // ---------------------------------------------------------------------------
  // 笛卡尔直线运动（下探 / 提起）
  // 对应 ROS1：computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory)
  // [ROS2] 接口基本一致，返回已完成比例 [0,1]
  // ---------------------------------------------------------------------------
  bool linearMove(const geometry_msgs::msg::Pose & start,
                  const geometry_msgs::msg::Pose & end,
                  const std::string & label)
  {
    RCLCPP_INFO(logger_, "--> Cartesian move: %s", label.c_str());

    std::vector<geometry_msgs::msg::Pose> waypoints{start, end};
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.005;
    const double jump_threshold = 0.0;  // 通常设 0 禁用跳变检测

    double fraction = move_group_.computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(logger_, "Cartesian path computed: %.2f%% achieved", fraction * 100.0);
    if (fraction < 0.9) {
      RCLCPP_WARN(logger_, "Cartesian %s only %.2f%%, falling back to free plan", label.c_str(), fraction * 100);
      return goToPose(end, label);
    }

    return move_group_.execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  // ---------------------------------------------------------------------------
  // 向规划场景中添加桌面 + 目标立方体 + 放置区域参考
  // 对应 ROS1：planning_scene_interface.applyCollisionObjects(...)
  // ---------------------------------------------------------------------------
  void addCollisionObjects()
  {
    const std::string frame = move_group_.getPlanningFrame();
    std::vector<moveit_msgs::msg::CollisionObject> objects;

    // --- 桌子 ---
    {
      moveit_msgs::msg::CollisionObject obj;
      obj.id = TABLE_ID;
      obj.header.frame_id = frame;

      shape_msgs::msg::SolidPrimitive shape;
      shape.type = shape_msgs::msg::SolidPrimitive::BOX;
      shape.dimensions = {TABLE_SX, TABLE_SY, TABLE_SZ};

      geometry_msgs::msg::Pose pose;
      pose.position.x = TABLE_X;
      pose.position.y = TABLE_Y;
      pose.position.z = TABLE_Z;
      pose.orientation.w = 1.0;

      obj.primitives.push_back(shape);
      obj.primitive_poses.push_back(pose);
      obj.operation = moveit_msgs::msg::CollisionObject::ADD;
      objects.push_back(obj);
    }

    // --- 目标立方体 ---
    {
      moveit_msgs::msg::CollisionObject obj;
      obj.id = TARGET_CUBE_ID;
      obj.header.frame_id = frame;

      shape_msgs::msg::SolidPrimitive shape;
      shape.type = shape_msgs::msg::SolidPrimitive::BOX;
      shape.dimensions = {CUBE_SIZE, CUBE_SIZE, CUBE_SIZE};

      geometry_msgs::msg::Pose pose;
      pose.position.x = CUBE_X;
      pose.position.y = CUBE_Y;
      pose.position.z = TABLE_Z + TABLE_SZ / 2.0 + CUBE_SIZE / 2.0;
      pose.orientation.w = 1.0;

      obj.primitives.push_back(shape);
      obj.primitive_poses.push_back(pose);
      obj.operation = moveit_msgs::msg::CollisionObject::ADD;
      objects.push_back(obj);
    }

    psi_.applyCollisionObjects(objects);
    RCLCPP_INFO(logger_, "Added %zu collision objects to the planning scene", objects.size());
  }

  // ---------------------------------------------------------------------------
  // 把立方体 "贴附" 到末端（attach / detach 只改变碰撞计算，不做力学仿真）
  // ---------------------------------------------------------------------------
  void attachCube()
  {
    RCLCPP_INFO(logger_, "Attaching cube to end-effector link: %s",
                move_group_.getEndEffectorLink().c_str());
    move_group_.attachObject(TARGET_CUBE_ID, move_group_.getEndEffectorLink());
    rclcpp::sleep_for(500ms);
  }

  void detachCube()
  {
    RCLCPP_INFO(logger_, "Detaching cube");
    move_group_.detachObject(TARGET_CUBE_ID);
    rclcpp::sleep_for(500ms);
  }

  // ---------------------------------------------------------------------------
  // 抓取姿态：让末端 Z 轴朝向世界 -Z（从上方垂直接近）
  // 绕 X 轴旋转 π → 默认工具朝上翻转为朝下
  // ---------------------------------------------------------------------------
  geometry_msgs::msg::Pose makeTopDownPose(double x, double y, double z) const
  {
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;

    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, 0.0);  // 末端朝下
    p.orientation = tf2::toMsg(q);
    return p;
  }

  // ---------------------------------------------------------------------------
  // 抓取序列：pre-grasp → grasp → attach → lift
  // ---------------------------------------------------------------------------
  void pick()
  {
    const double cube_top_z = TABLE_Z + TABLE_SZ / 2.0 + CUBE_SIZE + GRASP_OFFSET;

    auto pre_grasp = makeTopDownPose(CUBE_X, CUBE_Y, cube_top_z + APPROACH_OFFSET);
    auto grasp     = makeTopDownPose(CUBE_X, CUBE_Y, cube_top_z);
    auto lift      = makeTopDownPose(CUBE_X, CUBE_Y, cube_top_z + APPROACH_OFFSET);

    if (!goToPose(pre_grasp, "pre_grasp")) return;
    if (!linearMove(pre_grasp, grasp, "descend")) return;

    attachCube();

    linearMove(grasp, lift, "lift");
  }

  // ---------------------------------------------------------------------------
  // 放置序列：pre-place → place → detach → retreat
  // ---------------------------------------------------------------------------
  void place()
  {
    const double place_top_z = TABLE_Z + TABLE_SZ / 2.0 + CUBE_SIZE + GRASP_OFFSET;

    auto pre_place = makeTopDownPose(PLACE_X, PLACE_Y, place_top_z + APPROACH_OFFSET);
    auto place     = makeTopDownPose(PLACE_X, PLACE_Y, place_top_z);
    auto retreat   = makeTopDownPose(PLACE_X, PLACE_Y, place_top_z + APPROACH_OFFSET);

    if (!goToPose(pre_place, "pre_place")) return;
    if (!linearMove(pre_place, place, "place_down")) return;

    detachCube();

    linearMove(place, retreat, "retreat");
  }

  // ---- 成员 ----
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface psi_;
};

// =============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // [ROS2] 允许从 launch 自动声明参数，这样 MoveGroupInterface 能读到 URDF/SRDF
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("pick_place_demo", node_options);

  // [ROS2] 必须在独立线程里 spin，不然 MoveGroupInterface 收不到状态/TF
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  try {
    PickPlaceDemo demo(node);
    demo.run();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Exception: %s", e.what());
  }

  rclcpp::shutdown();
  if (spinner.joinable()) spinner.join();
  return 0;
}
