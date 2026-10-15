#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/utils/moveit_error_code.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "tracer_piper_color_grasp/action/pick_object.hpp"
#include "tracer_piper_color_grasp/action/place_object.hpp"
#include "tracer_piper_color_grasp/color_pose_estimator.hpp"

namespace tracer_piper_color_grasp
{

namespace
{

using FollowJointTrajectory =
  control_msgs::action::FollowJointTrajectory;
using GripperGoalHandle =
  rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

constexpr uint8_t kSuccess = 0;
constexpr uint8_t kInvalidColor = 1;
constexpr uint8_t kNotReady = 2;
constexpr uint8_t kDetectionFailed = 3;
constexpr uint8_t kTargetOutsideWorkspace = 4;
constexpr uint8_t kPlanningFailed = 5;
constexpr uint8_t kGripperFailed = 6;
constexpr uint8_t kCanceledOrStopped = 7;
constexpr uint8_t kPlacementFailed = 8;

template<typename T>
T parameterOrDeclare(
  rclcpp::Node * node, const std::string & name, const T & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}

std::string normalizeColor(std::string color)
{
  std::transform(
    color.begin(), color.end(), color.begin(),
    [](unsigned char value) {return static_cast<char>(std::tolower(value));});
  if (color == "red" || color == "红" || color == "红色") {
    return "red";
  }
  if (color == "yellow" || color == "黄" || color == "黄色") {
    return "yellow";
  }
  if (color == "green" || color == "绿" || color == "绿色") {
    return "green";
  }
  return "";
}

class MotionExecutor
{
public:
  explicit MotionExecutor(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    planning_group_(parameterOrDeclare<std::string>(node_.get(),
        "planning_group", "piper_arm")),
    end_effector_link_(parameterOrDeclare<std::string>(node_.get(),
        "end_effector_link", "piper_gripper_base")),
    reference_frame_(node_->get_parameter("pose_reference_frame").as_string()),
    gripper_action_name_(parameterOrDeclare<std::string>(node_.get(),
        "gripper_action_name",
        "/piper_gripper_controller/follow_joint_trajectory")),
    server_timeout_s_(parameterOrDeclare<double>(node_.get(),
        "motion.server_timeout_s", 10.0)),
    planning_time_s_(parameterOrDeclare<double>(node_.get(),
        "motion.planning_time_s", 8.0)),
    planning_attempts_(parameterOrDeclare<int>(node_.get(),
        "motion.planning_attempts", 8)),
    velocity_scaling_(parameterOrDeclare<double>(node_.get(),
        "motion.velocity_scaling", 0.15)),
    acceleration_scaling_(parameterOrDeclare<double>(node_.get(),
        "motion.acceleration_scaling", 0.10)),
    cartesian_step_m_(parameterOrDeclare<double>(node_.get(),
        "motion.cartesian_step_m", 0.005)),
    cartesian_min_fraction_(parameterOrDeclare<double>(node_.get(),
        "motion.cartesian_min_fraction", 0.95)),
    gripper_motion_s_(parameterOrDeclare<double>(node_.get(),
        "motion.gripper_motion_s", 2.0)),
    gripper_timeout_s_(parameterOrDeclare<double>(node_.get(),
        "motion.gripper_timeout_s", 15.0))
  {
    gripper_client_ =
      rclcpp_action::create_client<FollowJointTrajectory>(
      node_, gripper_action_name_);
  }

  bool ensureReady(std::string & error)
  {
    if (move_group_ && planning_scene_) {
      if (gripper_client_->wait_for_action_server(
          std::chrono::duration<double>(server_timeout_s_)))
      {
        error.clear();
        return true;
      }
      error = "夹爪 FollowJointTrajectory Action 未就绪";
      return false;
    }

    try {
      moveit::planning_interface::MoveGroupInterface::Options options(
        planning_group_, "robot_description", "");
      move_group_ =
        std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        node_, options, nullptr,
        rclcpp::Duration::from_seconds(server_timeout_s_));
      if (!move_group_->setEndEffectorLink(end_effector_link_)) {
        error = "MoveIt 无法设置末端链接 " + end_effector_link_;
        move_group_.reset();
        return false;
      }
      move_group_->setPoseReferenceFrame(reference_frame_);
      move_group_->setPlanningPipelineId("ompl");
      move_group_->setPlannerId("RRTConnectkConfigDefault");
      move_group_->setPlanningTime(planning_time_s_);
      move_group_->setNumPlanningAttempts(
        static_cast<unsigned int>(std::max(1, planning_attempts_)));
      move_group_->allowReplanning(true);
      move_group_->startStateMonitor(2.0);
      planning_scene_ =
        std::make_unique<
        moveit::planning_interface::PlanningSceneInterface>();
    } catch (const std::exception & exception) {
      error = std::string("连接 MoveIt 失败：") + exception.what();
      move_group_.reset();
      planning_scene_.reset();
      return false;
    }

    if (!gripper_client_->wait_for_action_server(
        std::chrono::duration<double>(server_timeout_s_)))
    {
      error = "夹爪 FollowJointTrajectory Action 未就绪";
      return false;
    }
    error.clear();
    return true;
  }

  bool moveToJoints(
    const std::vector<double> & joints,
    const std::function<bool()> & stop_requested,
    std::string & error)
  {
    if (stop_requested()) {
      error = "运动开始前收到取消或急停";
      return false;
    }
    if (!move_group_ || joints.size() != 6U) {
      error = "MoveIt 未就绪或关节目标不是 6 维";
      return false;
    }

    configurePlanner();
    if (!move_group_->setJointValueTarget(joints)) {
      error = "关节目标超出限制";
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group_->plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      error = "关节轨迹规划失败，错误码 " +
        std::to_string(plan_result.val);
      return false;
    }
    if (stop_requested()) {
      error = "关节轨迹执行前收到取消或急停";
      return false;
    }
    const auto execute_result = move_group_->execute(plan);
    if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
      error = "关节轨迹执行失败，错误码 " +
        std::to_string(execute_result.val);
      return false;
    }
    error.clear();
    return true;
  }

  bool moveToPose(
    const geometry_msgs::msg::PoseStamped & target,
    const std::function<bool()> & stop_requested,
    std::string & error)
  {
    if (stop_requested()) {
      error = "位姿运动开始前收到取消或急停";
      return false;
    }
    if (!move_group_) {
      error = "MoveIt 未就绪";
      return false;
    }

    configurePlanner();
    if (!move_group_->setPoseTarget(target, end_effector_link_)) {
      error = "目标位姿没有有效 IK 解";
      move_group_->clearPoseTargets();
      return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group_->plan(plan);
    move_group_->clearPoseTargets();
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      error = "位姿轨迹规划失败，错误码 " +
        std::to_string(plan_result.val);
      return false;
    }
    if (stop_requested()) {
      error = "位姿轨迹执行前收到取消或急停";
      return false;
    }
    const auto execute_result = move_group_->execute(plan);
    if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
      error = "位姿轨迹执行失败，错误码 " +
        std::to_string(execute_result.val);
      return false;
    }
    error.clear();
    return true;
  }

  bool moveCartesianTo(
    const geometry_msgs::msg::Pose & target,
    const std::function<bool()> & stop_requested,
    std::string & error)
  {
    if (stop_requested()) {
      error = "笛卡尔运动开始前收到取消或急停";
      return false;
    }
    if (!move_group_) {
      error = "MoveIt 未就绪";
      return false;
    }

    configurePlanner();
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = move_group_->computeCartesianPath(
      std::vector<geometry_msgs::msg::Pose>{target},
      cartesian_step_m_, 0.0, trajectory, true);
    if (fraction < cartesian_min_fraction_) {
      error = "笛卡尔路径完整率不足：" +
        std::to_string(fraction * 100.0) + "%";
      return false;
    }
    if (stop_requested()) {
      error = "笛卡尔轨迹执行前收到取消或急停";
      return false;
    }

    // Humble can execute a RobotTrajectory directly.  This also avoids
    // depending on the distro-specific MoveGroupInterface::Plan member names
    // (Humble uses trajectory_, Jazzy uses trajectory).
    const auto execute_result = move_group_->execute(trajectory);
    if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
      error = "笛卡尔轨迹执行失败，错误码 " +
        std::to_string(execute_result.val);
      return false;
    }
    error.clear();
    return true;
  }

  geometry_msgs::msg::PoseStamped currentPose() const
  {
    if (!move_group_) {
      return geometry_msgs::msg::PoseStamped();
    }
    return move_group_->getCurrentPose(end_effector_link_);
  }

  bool sendGripper(
    double opening_mm,
    const std::function<bool()> & stop_requested,
    std::string & error)
  {
    if (stop_requested()) {
      error = "夹爪动作开始前收到取消或急停";
      return false;
    }
    if (!gripper_client_->action_server_is_ready()) {
      error = "夹爪控制器未就绪";
      return false;
    }

    const double bounded_mm = std::clamp(opening_mm, 0.0, 100.0);
    const double half_opening_m = bounded_mm / 2000.0;
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = {
      "piper_gripper_joint1", "piper_gripper_joint2"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {half_opening_m, -half_opening_m};
    point.time_from_start =
      rclcpp::Duration::from_seconds(gripper_motion_s_);
    goal.trajectory.points.push_back(point);

    auto abandoned = std::make_shared<std::atomic_bool>(false);
    rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
    options.goal_response_callback = [this, abandoned](GripperGoalHandle::SharedPtr handle) {
        if (!handle) {return;}
        std::lock_guard<std::mutex> lock(gripper_mutex_);
        if (abandoned->load()) {
          gripper_client_->async_cancel_goal(handle);
        } else {
          gripper_goal_ = handle;
        }
      };
    auto goal_future = gripper_client_->async_send_goal(goal, options);
    const auto accept_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(gripper_timeout_s_);
    while (goal_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
      if (stop_requested() || std::chrono::steady_clock::now() >= accept_deadline) {
        abandoned->store(true);
        stop();
        error = stop_requested() ? "发送夹爪目标时被取消或急停" : "发送夹爪目标超时";
        return false;
      }
    }
    if (stop_requested()) {
      abandoned->store(true);
      stop();
      error = "夹爪目标接受时被取消或急停";
      return false;
    }
    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      error = "夹爪目标被控制器拒绝";
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(gripper_mutex_);
      gripper_goal_ = goal_handle;
    }

    auto result_future = gripper_client_->async_get_result(goal_handle);
    const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration<double>(gripper_timeout_s_);
    while (result_future.wait_for(std::chrono::milliseconds(50)) !=
      std::future_status::ready)
    {
      if (stop_requested() || std::chrono::steady_clock::now() >= deadline) {
        gripper_client_->async_cancel_goal(goal_handle);
        {
          std::lock_guard<std::mutex> lock(gripper_mutex_);
          gripper_goal_.reset();
        }
        error = stop_requested() ?
          "夹爪动作被取消或急停" : "等待夹爪结果超时";
        return false;
      }
    }

    const auto wrapped_result = result_future.get();
    {
      std::lock_guard<std::mutex> lock(gripper_mutex_);
      gripper_goal_.reset();
    }
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !wrapped_result.result ||
      wrapped_result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL)
    {
      error = "夹爪动作执行失败";
      return false;
    }
    error.clear();
    return true;
  }

  bool attachCube(
    const std::string & object_id,
    const geometry_msgs::msg::PoseStamped & cube_center,
    double cube_size_m,
    std::string & error)
  {
    if (!move_group_ || !planning_scene_) {
      error = "MoveIt PlanningScene 未就绪";
      return false;
    }

    moveit_msgs::msg::CollisionObject object;
    object.header = cube_center.header;
    object.id = object_id;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {cube_size_m, cube_size_m, cube_size_m};
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(cube_center.pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    if (!planning_scene_->applyCollisionObject(object)) {
      error = "无法将目标方块加入 MoveIt PlanningScene";
      return false;
    }

    const std::vector<std::string> touch_links = {
      end_effector_link_,
      "piper_gripper_link1",
      "piper_gripper_link2"
    };
    if (!move_group_->attachObject(
        object_id, end_effector_link_, touch_links))
    {
      planning_scene_->removeCollisionObjects({object_id});
      error = "无法将目标方块附着到夹爪规划模型";
      return false;
    }
    error.clear();
    return true;
  }

  void detachAndRemoveCube(const std::string & object_id)
  {
    if (move_group_) {
      move_group_->detachObject(object_id);
    }
    if (planning_scene_) {
      planning_scene_->removeCollisionObjects({object_id});
    }
  }

  void stop()
  {
    if (move_group_) {
      move_group_->stop();
      move_group_->clearPoseTargets();
    }
    GripperGoalHandle::SharedPtr gripper_goal;
    {
      std::lock_guard<std::mutex> lock(gripper_mutex_);
      gripper_goal = gripper_goal_;
      gripper_goal_.reset();
    }
    if (gripper_goal) {
      gripper_client_->async_cancel_goal(gripper_goal);
    }
  }

private:
  void configurePlanner()
  {
    move_group_->setPlanningTime(planning_time_s_);
    move_group_->setNumPlanningAttempts(
      static_cast<unsigned int>(std::max(1, planning_attempts_)));
    move_group_->setMaxVelocityScalingFactor(
      std::clamp(velocity_scaling_, 0.01, 1.0));
    move_group_->setMaxAccelerationScalingFactor(
      std::clamp(acceleration_scaling_, 0.01, 1.0));
    move_group_->setStartStateToCurrentState();
  }

  rclcpp::Node::SharedPtr node_;
  std::string planning_group_;
  std::string end_effector_link_;
  std::string reference_frame_;
  std::string gripper_action_name_;
  double server_timeout_s_;
  double planning_time_s_;
  int planning_attempts_;
  double velocity_scaling_;
  double acceleration_scaling_;
  double cartesian_step_m_;
  double cartesian_min_fraction_;
  double gripper_motion_s_;
  double gripper_timeout_s_;

  std::unique_ptr<
    moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<
    moveit::planning_interface::PlanningSceneInterface> planning_scene_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_client_;
  std::mutex gripper_mutex_;
  GripperGoalHandle::SharedPtr gripper_goal_;
};

}  // namespace

class ColorGraspServer : public rclcpp::Node
{
public:
  using PickObject = action::PickObject;
  using PlaceObject = action::PlaceObject;
  using PickGoalHandle = rclcpp_action::ServerGoalHandle<PickObject>;
  using PlaceGoalHandle = rclcpp_action::ServerGoalHandle<PlaceObject>;

  ColorGraspServer()
  : Node(
      "color_grasp_server",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    busy_(false),
    cancel_requested_(false),
    emergency_stop_(true),
    has_payload_(false),
    payload_attached_(false),
    odometry_seen_(false),
    base_linear_speed_mps_(0.0),
    base_angular_speed_rps_(0.0)
  {
    estimator_ = std::make_unique<ColorPoseEstimator>(this, &tf_buffer_);
    reference_frame_ =
      get_parameter("pose_reference_frame").as_string();
    cube_size_m_ = parameterOrDeclare<double>(this,
      "grasp.cube_size_m", 0.05);
    tcp_to_grasp_center_m_ = parameterOrDeclare<double>(this,
      "grasp.tcp_to_grasp_center_m", 0.138);
    grasp_offset_x_m_ = parameterOrDeclare<double>(this,
      "grasp.offset_x_m", 0.0);
    grasp_offset_y_m_ = parameterOrDeclare<double>(this,
      "grasp.offset_y_m", 0.0);
    grasp_offset_z_m_ = parameterOrDeclare<double>(this,
      "grasp.offset_z_m", 0.0);
    pregrasp_height_m_ = parameterOrDeclare<double>(this,
      "grasp.pregrasp_height_m", 0.12);
    lift_height_m_ = parameterOrDeclare<double>(this,
      "grasp.lift_height_m", 0.12);
    gripper_open_mm_ = parameterOrDeclare<double>(this,
      "grasp.gripper_open_mm", 90.0);
    gripper_close_mm_ = parameterOrDeclare<double>(this,
      "grasp.gripper_close_mm", 46.0);
    min_x_m_ = parameterOrDeclare<double>(this,
      "workspace.min_x_m", 0.20);
    max_x_m_ = parameterOrDeclare<double>(this,
      "workspace.max_x_m", 0.65);
    min_y_m_ = parameterOrDeclare<double>(this,
      "workspace.min_y_m", -0.30);
    max_y_m_ = parameterOrDeclare<double>(this,
      "workspace.max_y_m", 0.30);
    min_z_m_ = parameterOrDeclare<double>(this,
      "workspace.min_z_m", -0.30);
    max_z_m_ = parameterOrDeclare<double>(this,
      "workspace.max_z_m", 0.10);

    home_joints_ = parameterOrDeclare<std::vector<double>>(this,
      "home_joint_values",
      {0.072925, 0.806721, -0.927252, -0.004518, 1.15, -0.026715});
    carry_joints_ = parameterOrDeclare<std::vector<double>>(this,
      "carry_joint_values", home_joints_);
    place_basket_frame_ = parameterOrDeclare<std::string>(this,
      "place.basket_frame", "map");
    place_basket_center_x_m_ = parameterOrDeclare<double>(this,
      "place.basket_center_x_m", -4.0);
    place_basket_center_y_m_ = parameterOrDeclare<double>(this,
      "place.basket_center_y_m", 4.0);
    place_finger_center_z_m_ = parameterOrDeclare<double>(this,
      "place.finger_center_z_m", 0.325);
    place_tf_timeout_s_ = parameterOrDeclare<double>(this,
      "place.tf_timeout_s", 1.0);
    place_release_orientation_ = parameterOrDeclare<std::vector<double>>(this,
      "place.release_orientation_xyzw",
      {-0.0508278333, 0.7529963848, 0.0236494598, 0.6556322741});
    if (place_basket_frame_.empty() ||
      !std::isfinite(place_basket_center_x_m_) ||
      !std::isfinite(place_basket_center_y_m_) ||
      !std::isfinite(place_finger_center_z_m_) ||
      !std::isfinite(place_tf_timeout_s_) || place_tf_timeout_s_ <= 0.0 ||
      place_release_orientation_.size() != 4U ||
      !std::all_of(
        place_release_orientation_.begin(),
        place_release_orientation_.end(),
        [](double value) {return std::isfinite(value);}))
    {
      throw std::invalid_argument(
              "PLACE 箱心、高度、TF 超时或释放姿态参数无效");
    }
    max_base_linear_speed_mps_ = parameterOrDeclare<double>(this,
      "safety.max_base_linear_speed_mps", 0.02);
    max_base_angular_speed_rps_ = parameterOrDeclare<double>(this,
      "safety.max_base_angular_speed_rps", 0.03);
    clock_check_wall_seconds_ = parameterOrDeclare<double>(this,
      "safety.clock_check_wall_seconds", 0.30);
    emergency_stop_.store(parameterOrDeclare<bool>(this,
      "safety.start_with_emergency_stop", true));

    payload_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/manipulation/has_payload", rclcpp::QoS(1).reliable().transient_local());

    busy_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/color_grasp/busy",
      rclcpp::QoS(1).reliable().transient_local());
    state_publisher_ = create_publisher<std_msgs::msg::String>(
      "/color_grasp/state",
      rclcpp::QoS(10));
    emergency_stop_subscription_ =
      create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop", rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::ConstSharedPtr message) {
        emergency_stop_.store(message->data);
        if (message->data) {
          cancel_requested_.store(true);
          if (motion_) {
            motion_->stop();
          }
          publishState("EMERGENCY_STOP");
        }
      });
    const auto base_odom_topic = parameterOrDeclare<std::string>(this,
      "base_odom_topic", "/wheel/odom");
    odometry_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
      base_odom_topic, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr message) {
        const auto & linear = message->twist.twist.linear;
        const auto & angular = message->twist.twist.angular;
        const double linear_speed = std::sqrt(
          linear.x * linear.x +
          linear.y * linear.y +
          linear.z * linear.z);
        const double angular_speed = std::sqrt(
          angular.x * angular.x +
          angular.y * angular.y +
          angular.z * angular.z);
        base_linear_speed_mps_.store(linear_speed);
        base_angular_speed_rps_.store(angular_speed);
        last_odometry_wall_ns_.store(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
        odometry_seen_.store(true);
        if (busy_.load() &&
          (!std::isfinite(linear_speed) || !std::isfinite(angular_speed) ||
          linear_speed >= max_base_linear_speed_mps_ ||
          angular_speed >= max_base_angular_speed_rps_))
        {
          cancel_requested_.store(true);
          if (motion_) {
            motion_->stop();
          }
          publishState("BASE_MOTION_STOP");
        }
      });

    pick_server_ = rclcpp_action::create_server<PickObject>(
      this, "/manipulation/pick",
      [this](const rclcpp_action::GoalUUID &,
        const std::shared_ptr<const PickObject::Goal> goal) {
        if (normalizeColor(goal->color).empty()) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return acceptGoal(true);
      },
      [this](const std::shared_ptr<PickGoalHandle> handle) {
        return handleCancel(handle);
      },
      [this](const std::shared_ptr<PickGoalHandle> handle) {
        handleAccepted(handle);
      });
    place_server_ = rclcpp_action::create_server<PlaceObject>(
      this, "/manipulation/place",
      [this](const rclcpp_action::GoalUUID &,
        const std::shared_ptr<const PlaceObject::Goal>) {
        return acceptGoal(false);
      },
      [this](const std::shared_ptr<PlaceGoalHandle> handle) {
        return handleCancel(handle);
      },
      [this](const std::shared_ptr<PlaceGoalHandle> handle) {
        handleAccepted(handle);
      });

    publishBusy(false);
    setPayload(false);
    publishState("WAITING_FOR_DEPENDENCIES");
  }

  ~ColorGraspServer() override
  {
    shutdownWorker();
  }

  void shutdownWorker()
  {
    cancel_requested_.store(true);
    if (motion_) {motion_->stop();}
    if (worker_.joinable()) {worker_.join();}
  }

  void initialize(const rclcpp::Node::SharedPtr & node)
  {
    motion_ = std::make_unique<MotionExecutor>(node);
  }

private:
  rclcpp_action::GoalResponse acceptGoal(bool pick)
  {
    if (pick == has_payload_.load()) {
      RCLCPP_WARN(get_logger(), "Rejected action: incompatible payload state.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (emergency_stop_.load()) {
      RCLCPP_WARN(get_logger(), "Rejected grasp command while emergency stop is active.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "Rejected grasp command because another task is active.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  template<typename ActionT>
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)
  {
    cancel_requested_.store(true);
    if (motion_) {
      motion_->stop();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  template<typename ActionT>
  void handleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
  {
    if (worker_.joinable()) {
      worker_.join();
    }
    cancel_requested_.store(false);
    publishBusy(true);
    worker_ = std::thread([this, goal_handle]() {
      try {
        execute(goal_handle);
      } catch (const std::exception & exception) {
        if (motion_) {motion_->stop();}
        finish(goal_handle, false, kPlanningFailed, exception.what());
      }
    });
  }

  template<typename ActionT>
  bool stopRequested(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle) const
  {
    return cancel_requested_.load() || emergency_stop_.load() ||
           goal_handle->is_canceling() || !rclcpp::ok();
  }

  void setPayload(bool value)
  {
    has_payload_.store(value);
    std_msgs::msg::Bool message;
    message.data = value;
    payload_publisher_->publish(message);
  }

  void publishBusy(bool value)
  {
    std_msgs::msg::Bool message;
    message.data = value;
    busy_publisher_->publish(message);
  }

  void publishState(const std::string & value)
  {
    std_msgs::msg::String message;
    message.data = value;
    state_publisher_->publish(message);
  }

  template<typename ActionT>
  void publishFeedback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle,
    const std::string & stage, float progress)
  {
    auto feedback = std::make_shared<typename ActionT::Feedback>();
    feedback->stage = stage;
    feedback->progress = std::clamp(progress, 0.0F, 1.0F);
    goal_handle->publish_feedback(feedback);
    publishState(stage);
  }

  template<typename ActionT>
  void finish(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle,
    bool success, uint8_t error_code,
    const std::string & message)
  {
    if (success && stopRequested(goal_handle)) {
      success = false;
      error_code = kCanceledOrStopped;
    }
    auto result = std::make_shared<typename ActionT::Result>();
    result->success = success;
    result->error_code = error_code;
    result->message = message;

    if (!success) {
      RCLCPP_ERROR(get_logger(), "Manipulation failed (%u): %s",
        static_cast<unsigned int>(error_code), message.c_str());
    }
    busy_.store(false);
    publishBusy(false);
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      publishState("CANCELED");
    } else if (success) {
      goal_handle->succeed(result);
      publishState("SUCCEEDED");
    } else {
      goal_handle->abort(result);
      publishState("FAILED");
    }
  }

  bool targetInsideWorkspace(
    const geometry_msgs::msg::Point & point) const
  {
    return point.x >= min_x_m_ && point.x <= max_x_m_ &&
           point.y >= min_y_m_ && point.y <= max_y_m_ &&
           point.z >= min_z_m_ && point.z <= max_z_m_;
  }

  bool baseIsStationary(std::string & error) const
  {
    if (!odometry_seen_.load()) {
      error = "尚未收到小车轮式里程计，拒绝启动机械臂";
      return false;
    }
    const auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    if (wall_ns - last_odometry_wall_ns_.load() > 1000000000LL ||
      !std::isfinite(base_linear_speed_mps_.load()) ||
      !std::isfinite(base_angular_speed_rps_.load()) ||
      base_linear_speed_mps_.load() >= max_base_linear_speed_mps_ ||
      base_angular_speed_rps_.load() >= max_base_angular_speed_rps_)
    {
      error = "小车仍在运动，请停车后重新发送抓取指令";
      return false;
    }
    error.clear();
    return true;
  }

  bool simulationClockAdvancing(
    const std::function<bool()> & stop_requested,
    std::string & error)
  {
    if (!get_parameter("use_sim_time").as_bool()) {
      return true;
    }
    const auto before = now();
    const auto wait_duration = std::chrono::duration<double>(
      std::max(0.10, clock_check_wall_seconds_));
    const auto deadline = std::chrono::steady_clock::now() + wait_duration;
    while (std::chrono::steady_clock::now() < deadline) {
      if (stop_requested()) {
        error = "任务已取消或急停";
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (now() <= before) {
      error = "Gazebo 仿真时钟没有推进，请先点击“继续”";
      return false;
    }
    error.clear();
    return true;
  }

  template<typename ActionT>
  void recoverHome(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle,
    bool carrying_object)
  {
    if (!motion_ || carrying_object || stopRequested(goal_handle)) {
      return;
    }
    std::string ignored;
    motion_->moveToJoints(
      home_joints_,
      [this, goal_handle]() {return stopRequested(goal_handle);},
      ignored);
  }

  void execute(const std::shared_ptr<PickGoalHandle> handle)
  {
    executePick(handle);
  }

  void execute(const std::shared_ptr<PlaceGoalHandle> handle)
  {
    executePlace(handle);
  }

  void executePlace(const std::shared_ptr<PlaceGoalHandle> goal_handle)
  {
    const auto should_stop =
      [this, goal_handle]() {return stopRequested(goal_handle);};
    std::string error;

    publishFeedback(goal_handle, "PLACE_CONNECTING", 0.10F);
    if (!motion_ || !motion_->ensureReady(error)) {
      finish(goal_handle, false, kNotReady,
        motion_ ? error : "抓放运动执行器尚未初始化");
      return;
    }
    if (!baseIsStationary(error)) {
      finish(goal_handle, false, kNotReady, error);
      return;
    }
    if (!simulationClockAdvancing(should_stop, error)) {
      finish(
        goal_handle, false,
        should_stop() ? kCanceledOrStopped : kNotReady, error);
      return;
    }

    // Use the robot's actual localized pose after Nav2 has stopped.  The
    // basket is fixed in map, but the robot is not guaranteed to stop at the
    // exact commanded navigation pose.
    geometry_msgs::msg::PointStamped basket_center_map;
    basket_center_map.header.frame_id = place_basket_frame_;
    basket_center_map.header.stamp = rclcpp::Time(0);
    basket_center_map.point.x = place_basket_center_x_m_;
    basket_center_map.point.y = place_basket_center_y_m_;
    basket_center_map.point.z = 0.0;

    geometry_msgs::msg::PointStamped basket_center_local;
    try {
      basket_center_local = tf_buffer_.transform(
        basket_center_map, reference_frame_,
        tf2::durationFromSec(place_tf_timeout_s_));
    } catch (const tf2::TransformException & exception) {
      finish(
        goal_handle, false, kPlacementFailed,
        "无法根据实际停车位姿将储物箱中心转换到 " +
        reference_frame_ + "：" + exception.what());
      return;
    }

    tf2::Quaternion release_orientation(
      place_release_orientation_[0], place_release_orientation_[1],
      place_release_orientation_[2], place_release_orientation_[3]);
    if (release_orientation.length2() < 1.0e-12) {
      finish(goal_handle, false, kPlacementFailed, "放置姿态四元数长度为零");
      return;
    }
    release_orientation.normalize();
    const tf2::Vector3 gripper_to_finger_center = tf2::quatRotate(
      release_orientation, tf2::Vector3(0.0, 0.0, tcp_to_grasp_center_m_));

    geometry_msgs::msg::PoseStamped release_pose;
    release_pose.header.frame_id = reference_frame_;
    release_pose.header.stamp = now();
    release_pose.pose.position.x =
      basket_center_local.point.x - gripper_to_finger_center.x();
    release_pose.pose.position.y =
      basket_center_local.point.y - gripper_to_finger_center.y();
    release_pose.pose.position.z =
      place_finger_center_z_m_ - gripper_to_finger_center.z();
    release_pose.pose.orientation = tf2::toMsg(release_orientation);

    RCLCPP_INFO(
      get_logger(),
      "Actual-pose basket alignment: basket %s=(%.3f, %.3f), "
      "finger center %s=(%.3f, %.3f, %.3f), gripper base "
      "target=(%.3f, %.3f, %.3f)",
      place_basket_frame_.c_str(),
      place_basket_center_x_m_, place_basket_center_y_m_,
      reference_frame_.c_str(),
      basket_center_local.point.x, basket_center_local.point.y,
      place_finger_center_z_m_,
      release_pose.pose.position.x,
      release_pose.pose.position.y,
      release_pose.pose.position.z);

    publishFeedback(goal_handle, "MOVING_ABOVE_STORAGE_BASKET", 0.30F);
    if (!motion_->moveToPose(release_pose, should_stop, error)) {
      finish(
        goal_handle, false,
        should_stop() ? kCanceledOrStopped : kPlacementFailed,
        "无法根据实际停车位姿移动到储物箱中心上方；继续保持夹持：" + error);
      return;
    }

    publishFeedback(goal_handle, "RELEASING_IN_STORAGE_BASKET", 0.60F);
    if (!motion_->sendGripper(gripper_open_mm_, should_stop, error)) {
      finish(
        goal_handle, false,
        should_stop() ? kCanceledOrStopped : kGripperFailed, error);
      return;
    }
    if (payload_attached_) {
      motion_->detachAndRemoveCube("color_grasp_target");
      payload_attached_ = false;
    }
    setPayload(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(600));

    if (goal_handle->get_goal()->return_home) {
      publishFeedback(goal_handle, "RETURNING_HOME", 0.85F);
    }
    if (goal_handle->get_goal()->return_home &&
      !motion_->moveToJoints(home_joints_, should_stop, error))
    {
      finish(
        goal_handle, false,
        should_stop() ? kCanceledOrStopped : kPlanningFailed,
        "物块已释放，但机械臂复位失败：" + error);
      return;
    }

    publishFeedback(goal_handle, "PLACE_COMPLETE", 1.0F);
    finish(goal_handle, true, kSuccess,
      goal_handle->get_goal()->return_home ?
      "物块已释放到 storage_basket，机械臂已复位" :
      "物块已释放到 storage_basket");
  }

  void executePick(const std::shared_ptr<PickGoalHandle> goal_handle)
  {
    const std::string color =
      normalizeColor(goal_handle->get_goal()->color);
    const auto should_stop =
      [this, goal_handle]() {return stopRequested(goal_handle);};
    std::string error;
    bool object_attached = false;
    const std::string object_id = "color_grasp_target";

    if (color.empty()) {
      finish(goal_handle, false, kInvalidColor, "颜色仅支持 red、yellow、green");
      return;
    }
    if (!motion_) {
      finish(goal_handle, false, kNotReady, "抓取运动执行器尚未初始化");
      return;
    }

    publishFeedback(goal_handle, "CONNECTING", 0.03F);
    if (!motion_->ensureReady(error)) {
      finish(goal_handle, false, kNotReady, error);
      return;
    }
    if (!baseIsStationary(error)) {
      finish(goal_handle, false, kNotReady, error);
      return;
    }
    if (!estimator_->cameraReady()) {
      finish(
        goal_handle, false, kNotReady,
        "腕部 RGB、深度图或 CameraInfo 尚未就绪");
      return;
    }
    if (!simulationClockAdvancing(should_stop, error)) {
      finish(
        goal_handle, false,
        should_stop() ? kCanceledOrStopped : kNotReady,
        error);
      return;
    }
    if (should_stop()) {
      finish(goal_handle, false, kCanceledOrStopped, "任务已取消或急停");
      return;
    }

    publishFeedback(goal_handle, "MOVING_TO_OBSERVATION", 0.10F);
    if (!motion_->moveToJoints(home_joints_, should_stop, error)) {
      finish(
        goal_handle, false,
        should_stop() ? kCanceledOrStopped : kPlanningFailed,
        error);
      return;
    }

    publishFeedback(goal_handle, "DETECTING_" + color, 0.20F);
    ColorDetection detection;
    if (!estimator_->waitForStableDetection(
        color, should_stop, detection, error))
    {
      recoverHome(goal_handle, false);
      finish(
        goal_handle, false,
        should_stop() ? kCanceledOrStopped : kDetectionFailed,
        error);
      return;
    }
    auto observation_pose = motion_->currentPose();
    if (observation_pose.header.frame_id.empty()) {
      finish(goal_handle, false, kNotReady, "无法读取机械臂当前末端位姿");
      return;
    }

    geometry_msgs::msg::PoseStamped grasp_pose;
    grasp_pose.header.frame_id = reference_frame_;
    grasp_pose.header.stamp = now();
    grasp_pose.pose.position = detection.surface_point.point;
    grasp_pose.pose.position.x += grasp_offset_x_m_;
    grasp_pose.pose.position.y += grasp_offset_y_m_;
    grasp_pose.pose.position.z +=
      tcp_to_grasp_center_m_ - cube_size_m_ * 0.5 + grasp_offset_z_m_;
    grasp_pose.pose.orientation = observation_pose.pose.orientation;

    RCLCPP_INFO(
      get_logger(),
      "Grasp calibration: vision=(%.4f, %.4f, %.4f), "
      "offset=(%.4f, %.4f, %.4f), commanded_tcp=(%.4f, %.4f, %.4f)",
      detection.surface_point.point.x,
      detection.surface_point.point.y,
      detection.surface_point.point.z,
      grasp_offset_x_m_, grasp_offset_y_m_, grasp_offset_z_m_,
      grasp_pose.pose.position.x,
      grasp_pose.pose.position.y,
      grasp_pose.pose.position.z);

    if (!targetInsideWorkspace(grasp_pose.pose.position)) {
      recoverHome(goal_handle, false);
      finish(
        goal_handle, false, kTargetOutsideWorkspace,
        "补偿后的抓取目标位于允许工作空间之外");
      return;
    }

    auto pregrasp_pose = grasp_pose;
    pregrasp_pose.pose.position.z += pregrasp_height_m_;

    publishFeedback(goal_handle, "OPENING_GRIPPER", 0.30F);
    if (!motion_->sendGripper(
        gripper_open_mm_, should_stop, error))
    {
      finish(
        goal_handle, false,
        should_stop() ? kCanceledOrStopped : kGripperFailed,
        error);
      return;
    }

    publishFeedback(goal_handle, "MOVING_TO_PREGRASP", 0.40F);
    if (!motion_->moveToPose(pregrasp_pose, should_stop, error)) {
      recoverHome(goal_handle, false);
      finish(
        goal_handle, false,
        should_stop() ? kCanceledOrStopped : kPlanningFailed,
        error);
      return;
    }

    publishFeedback(goal_handle, "DESCENDING_TO_CUBE", 0.48F);
    if (!motion_->moveCartesianTo(
        grasp_pose.pose, should_stop, error))
    {
      recoverHome(goal_handle, false);
      finish(
        goal_handle, false,
        should_stop() ? kCanceledOrStopped : kPlanningFailed,
        error);
      return;
    }

    publishFeedback(goal_handle, "CLOSING_GRIPPER", 0.55F);
    if (!motion_->sendGripper(
        gripper_close_mm_, should_stop, error))
    {
      finish(
        goal_handle, false,
        should_stop() ? kCanceledOrStopped : kGripperFailed,
        error);
      return;
    }

    geometry_msgs::msg::PoseStamped cube_center;
    cube_center.header = detection.surface_point.header;
    cube_center.pose.position = detection.surface_point.point;
    cube_center.pose.position.z -= cube_size_m_ * 0.5;
    cube_center.pose.orientation.w = 1.0;
    if (!motion_->attachCube(
        object_id, cube_center, cube_size_m_, error))
    {
      RCLCPP_WARN(
        get_logger(),
        "Continuing physical grasp without PlanningScene attachment: %s",
        error.c_str());
    } else {
      object_attached = true;
    }
    // From this point onward cancellation or planning failure must not cause
    // an automatic release.  PLACE is the only operation that opens the
    // gripper after a successful close.
    setPayload(true);
    payload_attached_ = object_attached;

    auto lift_pose = grasp_pose;
    lift_pose.pose.position.z += lift_height_m_;
    publishFeedback(goal_handle, "LIFTING_CUBE", 0.63F);
    if (!motion_->moveCartesianTo(
        lift_pose.pose, should_stop, error))
    {
      finish(
        goal_handle, false,
        should_stop() ? kCanceledOrStopped : kPlanningFailed,
        "方块抬升失败；保持当前位置：" + error);
      return;
    }

    // Keep the configured carry posture and closed gripper until PLACE.
    publishFeedback(goal_handle, "MOVING_TO_CARRY", 0.80F);
    if (!motion_->moveToJoints(
        carry_joints_, should_stop, error))
    {
      finish(
        goal_handle, false,
        should_stop() ? kCanceledOrStopped : kPlacementFailed,
        "无法进入安全携物姿态；保持夹持：" + error);
      return;
    }

    publishFeedback(goal_handle, "PICK_COMPLETE_HOLDING", 1.0F);
    finish(
      goal_handle, true, kSuccess,
      color + " 方块已抓取，夹爪保持闭合并进入安全携物姿态");
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<ColorPoseEstimator> estimator_;
  std::unique_ptr<MotionExecutor> motion_;

  rclcpp_action::Server<PickObject>::SharedPtr pick_server_;
  rclcpp_action::Server<PlaceObject>::SharedPtr place_server_;
  std::thread worker_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    emergency_stop_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odometry_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr busy_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr payload_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;

  std::atomic_bool busy_;
  std::atomic_bool cancel_requested_;
  std::atomic_bool emergency_stop_;
  std::atomic_bool has_payload_;
  bool payload_attached_;
  std::atomic_bool odometry_seen_;
  std::atomic<int64_t> last_odometry_wall_ns_{0};
  std::atomic<double> base_linear_speed_mps_;
  std::atomic<double> base_angular_speed_rps_;

  std::string reference_frame_;
  double cube_size_m_;
  double tcp_to_grasp_center_m_;
  double grasp_offset_x_m_;
  double grasp_offset_y_m_;
  double grasp_offset_z_m_;
  double pregrasp_height_m_;
  double lift_height_m_;
  double gripper_open_mm_;
  double gripper_close_mm_;
  double min_x_m_;
  double max_x_m_;
  double min_y_m_;
  double max_y_m_;
  double min_z_m_;
  double max_z_m_;
  std::vector<double> home_joints_;
  std::vector<double> carry_joints_;
  std::string place_basket_frame_;
  double place_basket_center_x_m_;
  double place_basket_center_y_m_;
  double place_finger_center_z_m_;
  double place_tf_timeout_s_;
  std::vector<double> place_release_orientation_;
  double max_base_linear_speed_mps_;
  double max_base_angular_speed_rps_;
  double clock_check_wall_seconds_;
};

}  // namespace tracer_piper_color_grasp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<tracer_piper_color_grasp::ColorGraspServer>();
  node->initialize(node);

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  node->shutdownWorker();
  rclcpp::shutdown();
  return 0;
}
