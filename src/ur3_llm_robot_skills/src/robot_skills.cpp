#include <chrono>
#include <algorithm>
#include <future>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
using Json = nlohmann::json;

namespace
{
struct Step
{
  std::string skill;
  std::string object;
  std::string zone;
};

const std::vector<std::string> kObjects = {"red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube"};
const std::vector<std::string> kZones = {"zone_a", "zone_b", "zone_c"};
const std::vector<std::string> kDestinations = {
  "zone_a", "zone_b", "zone_c", "temp_1", "temp_2"};
constexpr double kContactInsetZ = 0.008;
constexpr double kApproachHeight = 0.15;
constexpr double kAttachLiftHeight = 0.06;
const std::vector<std::string> kTouchLinks = {
  "gripper_base_link", "left_finger_link", "right_finger_link", "gripper_tcp",
  "tool0", "wrist_3_link"};

bool decode_plan(const std::string & serialized, std::vector<Step> & steps, std::string & reason)
{
  try {
    const auto root = Json::parse(serialized);
    if (!root.is_object() || root.size() != 1 || !root.contains("plan") || !root["plan"].is_array()) {
      reason = "Expected an object containing only a plan array";
      return false;
    }
    bool carrying = false;
    std::string carried_object;
    const auto & plan = root["plan"];
    if (plan.empty()) {
      reason = "Plan is empty";
      return false;
    }
    for (std::size_t index = 0; index < plan.size(); ++index) {
      const auto & item = plan[index];
      if (!item.is_object() || !item.contains("skill") || !item["skill"].is_string()) {
        reason = "Every step must contain a string skill";
        return false;
      }
      const std::string skill = item["skill"].get<std::string>();
      if (skill == "pick") {
        if (item.size() != 2 || !item.contains("object") || !item["object"].is_string() || carrying) {
          reason = "Invalid pick step or object already held";
          return false;
        }
        const auto object = item["object"].get<std::string>();
        if (std::find(kObjects.begin(), kObjects.end(), object) == kObjects.end()) {
          reason = "Unknown object: " + object;
          return false;
        }
        carrying = true;
        carried_object = object;
        steps.push_back({skill, object, ""});
      } else if (skill == "place") {
        if (item.size() != 3 || !item.contains("object") || !item["object"].is_string() ||
          !item.contains("zone") || !item["zone"].is_string())
        {
          reason = "Invalid place step";
          return false;
        }
        const auto object = item["object"].get<std::string>();
        const auto zone = item["zone"].get<std::string>();
        if (std::find(kObjects.begin(), kObjects.end(), object) == kObjects.end() ||
          std::find(kDestinations.begin(), kDestinations.end(), zone) == kDestinations.end())
        {
          reason = "Unknown object or zone";
          return false;
        }
        if (!carrying || carried_object != object) {
          reason = "place must match the currently picked object";
          return false;
        }
        carrying = false;
        steps.push_back({skill, object, zone});
      } else if (skill == "home") {
        if (item.size() != 1 || carrying) {
          reason = "home must be parameterless and executed with an empty gripper";
          return false;
        }
        steps.push_back({skill, "", ""});
      } else {
        reason = "Unsupported skill: " + skill;
        return false;
      }
    }
    if (carrying || steps.empty() || steps.back().skill != "home") {
      reason = "Plan must end with home and finish with an empty gripper";
      return false;
    }
    return true;
  } catch (const Json::exception & error) {
    reason = std::string("Invalid JSON plan: ") + error.what();
    return false;
  }
}

}  // namespace

class RobotSkills : public rclcpp::Node
{
public:
  explicit RobotSkills(const rclcpp::NodeOptions & options)
  : Node("ur3_skill_executor", options)
  {
    const auto declare_if_missing = [this](const auto & name, const auto & default_value) {
        if (!has_parameter(name)) {
          declare_parameter(name, default_value);
        }
      };
    declare_if_missing("planning_group", std::string("ur_manipulator"));
    declare_if_missing("planning_frame", std::string("base_link"));
    declare_if_missing("tool_link", std::string("gripper_tcp"));
    declare_if_missing("scene.table_size", std::vector<double>{0.14, 0.80, 0.05});
    declare_if_missing("scene.table_pose", std::vector<double>{-0.30, 0.0, 0.225});
    declare_if_missing("scene.object_size", std::vector<double>{0.04, 0.04, 0.04});
    declare_if_missing("scene.red_cube", std::vector<double>{-0.30, 0.05, 0.27});
    declare_if_missing("scene.yellow_cube", std::vector<double>{-0.30, 0.20, 0.27});
    declare_if_missing("scene.blue_cube", std::vector<double>{-0.25, -0.12, 0.27});
    declare_if_missing("scene.green_cube", std::vector<double>{-0.25, -0.30, 0.27});
    declare_if_missing("scene.purple_cube", std::vector<double>{-0.30, 0.35, 0.27});
    declare_if_missing("scene.zone_a", std::vector<double>{-0.25, -0.30, 0.27});
    declare_if_missing("scene.zone_b", std::vector<double>{-0.25, -0.12, 0.27});
    declare_if_missing("scene.zone_c", std::vector<double>{-0.25, 0.28, 0.27});
    declare_if_missing("scene.temp_1", std::vector<double>{-0.30, -0.20, 0.27});
    declare_if_missing("scene.temp_2", std::vector<double>{-0.30, -0.05, 0.27});

    status_publisher_ = create_publisher<std_msgs::msg::String>("/llm_task/execution_status", 10);
    plan_subscription_ = create_subscription<std_msgs::msg::String>(
      "/llm_task/validated_plan", 10,
      std::bind(&RobotSkills::execute_plan, this, std::placeholders::_1));
    
    scene_state_subscription_ = create_subscription<std_msgs::msg::String>(
      "/scene_state", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (!is_executing_plan_.load()) {
          try {
            const auto root = Json::parse(msg->data);
            std::lock_guard<std::mutex> lock(positions_mutex_);
            if (root.contains("objects") && root["objects"].is_object()) {
              for (auto& [object_id, observation] : root["objects"].items()) {
                if (observation.is_object() && observation.contains("xyz") &&
                  observation["xyz"].is_array() && observation["xyz"].size() == 3)
                {
                  object_positions_[object_id] = observation["xyz"].get<std::vector<double>>();
                }
              }
            }
          } catch (const Json::exception&) {
            // Ignore parse errors
          }
        }
      });

    gripper_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    gripper_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/gripper_controller/follow_joint_trajectory", gripper_callback_group_);
  }

  bool initialize()
  {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), get_parameter("planning_group").as_string());
    move_group_->setPoseReferenceFrame(get_parameter("planning_frame").as_string());
    move_group_->setEndEffectorLink(get_parameter("tool_link").as_string());
    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(0.40);
    move_group_->setMaxAccelerationScalingFactor(0.40);
    if (!move_group_->startStateMonitor(10.0) || !move_group_->getCurrentState(10.0)) {
      RCLCPP_ERROR(get_logger(), "No current robot state; check /joint_states and MoveIt.");
      return false;
    }
    if (!gripper_client_->wait_for_action_server(30s)) {
      RCLCPP_ERROR(get_logger(), "Gripper trajectory action is not available.");
      return false;
    }
    if (!initialize_scene()) {
      return false;
    }
    return true;
  }

private:
  bool initialize_scene()
  {
    const auto table_size = get_parameter("scene.table_size").as_double_array();
    const auto table_pose = get_parameter("scene.table_pose").as_double_array();
    if (table_size.size() != 3 || table_pose.size() != 3) {
      RCLCPP_ERROR(get_logger(), "scene.table_size and scene.table_pose must each have 3 values.");
      return false;
    }
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = get_parameter("planning_frame").as_string();
    table.id = "work_table";
    shape_msgs::msg::SolidPrimitive table_box;
    table_box.type = shape_msgs::msg::SolidPrimitive::BOX;
    table_box.dimensions = {table_size[0], table_size[1], table_size[2]};
    geometry_msgs::msg::Pose table_origin;
    table_origin.position.x = table_pose[0];
    table_origin.position.y = table_pose[1];
    table_origin.position.z = table_pose[2];
    table_origin.orientation.w = 1.0;
    table.primitives.push_back(table_box);
    table.primitive_poses.push_back(table_origin);
    table.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_objects.push_back(table);

    {
      std::lock_guard<std::mutex> lock(positions_mutex_);
      for (const auto & destination_id : kDestinations) {
        const auto destination_pose = get_parameter("scene." + destination_id).as_double_array();
        destination_positions_[destination_id] = destination_pose;
      }
      for (const auto & object_id : kObjects) {
        const auto object_pose = get_parameter("scene." + object_id).as_double_array();
        const auto object_size = get_parameter("scene.object_size").as_double_array();
        if (object_pose.size() != 3 || object_size.size() != 3) {
          RCLCPP_ERROR(get_logger(), "Object pose and size parameters must each have 3 values.");
          return false;
        }
        object_positions_[object_id] = object_pose;
        collision_objects.push_back(make_box(object_id, object_pose, object_size));
      }
    }

    return planning_scene_.applyCollisionObjects(collision_objects);
  }

  moveit_msgs::msg::CollisionObject make_box(
    const std::string & id, const std::vector<double> & xyz, const std::vector<double> & size) const
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = get_parameter("planning_frame").as_string();
    object.id = id;
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {size[0], size[1], size[2]};
    geometry_msgs::msg::Pose pose;
    pose.position.x = xyz[0];
    pose.position.y = xyz[1];
    pose.position.z = xyz[2];
    pose.orientation.w = 1.0;
    object.primitives.push_back(box);
    object.primitive_poses.push_back(pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    return object;
  }

  bool command_gripper(double position)
  {
    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = {"left_finger_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {position};
    point.time_from_start = rclcpp::Duration(2s);
    goal.trajectory.points.push_back(point);
    auto goal_future = gripper_client_->async_send_goal(goal);
    if (goal_future.wait_for(3s) != std::future_status::ready) {
      return false;
    }
    const auto goal_handle = goal_future.get();
    if (!goal_handle) {
      return false;
    }
    auto result_future = gripper_client_->async_get_result(goal_handle);
    if (result_future.wait_for(5s) != std::future_status::ready) {
      return false;
    }
    const auto code = result_future.get().code;
    return code == rclcpp_action::ResultCode::SUCCEEDED ||
           (position > 0.0 && code == rclcpp_action::ResultCode::ABORTED);
  }

  geometry_msgs::msg::Pose target_pose(const std::vector<double> & xyz) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = xyz[0];
    pose.position.y = xyz[1];
    pose.position.z = xyz[2];
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    return pose;
  }

  bool move_to_pose(const geometry_msgs::msg::Pose & pose, const std::string & label)
  {
    move_group_->setStartStateToCurrentState();
    move_group_->clearPoseTargets();
    if (!move_group_->setPoseTarget(pose)) {
      RCLCPP_ERROR(get_logger(), "Could not set pose target for %s.", label.c_str());
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group_->plan(plan);
    move_group_->clearPoseTargets();
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "MoveIt failed to plan %s (code %d).", label.c_str(), plan_result.val);
      return false;
    }

    const auto exec_result = move_group_->execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "MoveIt failed to execute %s (code %d).", label.c_str(), exec_result.val);
      return false;
    }
    return true;
  }

  bool move_home()
  {
    const std::map<std::string, double> ready = {
      {"shoulder_pan_joint", 0.0}, {"shoulder_lift_joint", -1.2},
      {"elbow_joint", -1.57}, {"wrist_1_joint", -1.57},
      {"wrist_2_joint", 1.57}, {"wrist_3_joint", 0.0}};
    move_group_->setStartStateToCurrentState();
    move_group_->clearPoseTargets();
    move_group_->setJointValueTarget(ready);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group_->plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "MoveIt failed to plan home (code %d).", plan_result.val);
      return false;
    }
    const auto exec_result = move_group_->execute(plan);
    move_group_->clearPoseTargets();
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "MoveIt failed to execute home (code %d).", exec_result.val);
      return false;
    }
    return true;
  }

  bool pick(const std::string & object_id)
  {
    std::vector<double> xyz;
    {
      std::lock_guard<std::mutex> lock(positions_mutex_);
      if (!held_object_.empty()) {
        RCLCPP_ERROR(get_logger(), "Object %s cannot be picked; %s is already held.",
          object_id.c_str(), held_object_.c_str());
        return false;
      }
      if (object_positions_.count(object_id)) {
        xyz = object_positions_.at(object_id);
      } else {
        xyz = get_parameter("scene." + object_id).as_double_array();
      }
    }

    if (xyz.size() != 3 || !command_gripper(0.0)) {
      return false;
    }
    auto approach = xyz;
    approach[2] += kApproachHeight;
    if (!move_to_pose(target_pose(approach), "approach " + object_id)) {
      return false;
    }
    planning_scene_.removeCollisionObjects({object_id});
    auto grasp_xyz = xyz;
    grasp_xyz[2] -= kContactInsetZ;
    if (!move_to_pose(target_pose(grasp_xyz), "grasp " + object_id)) {
      planning_scene_.applyCollisionObjects({make_box(
        object_id, xyz, get_parameter("scene.object_size").as_double_array())});
      return false;
    }
    if (!command_gripper(0.026)) {
      command_gripper(0.0);
      planning_scene_.applyCollisionObjects({make_box(
        object_id, xyz, get_parameter("scene.object_size").as_double_array())});
      return false;
    }
    auto lifted_xyz = xyz;
    lifted_xyz[2] += kAttachLiftHeight;
    auto lifted_tcp = grasp_xyz;
    lifted_tcp[2] += kAttachLiftHeight;
    if (!move_to_pose(target_pose(lifted_tcp), "initial lift " + object_id)) {
      command_gripper(0.0);
      planning_scene_.applyCollisionObjects({make_box(
        object_id, xyz, get_parameter("scene.object_size").as_double_array())});
      return false;
    }
    const auto object_size = get_parameter("scene.object_size").as_double_array();
    if (object_size.size() != 3 || !planning_scene_.applyCollisionObjects({
        make_box(object_id, lifted_xyz, object_size)}))
    {
      command_gripper(0.0);
      return false;
    }
    if (!move_group_->attachObject(object_id, get_parameter("tool_link").as_string(), kTouchLinks)) {
      command_gripper(0.0);
      planning_scene_.applyCollisionObjects({make_box(
        object_id, lifted_xyz, object_size)});
      return false;
    }
    if (!move_to_pose(target_pose(approach), "lift " + object_id)) {
      command_gripper(0.0);
      move_group_->detachObject(object_id);
      planning_scene_.applyCollisionObjects({make_box(
        object_id, lifted_xyz, object_size)});
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(positions_mutex_);
      held_object_ = object_id;
    }
    return true;
  }

  bool place(const std::string & object_id, const std::string & zone_id)
  {
    std::vector<double> xyz;
    {
      std::lock_guard<std::mutex> lock(positions_mutex_);
      if (held_object_.empty()) {
        RCLCPP_ERROR(get_logger(), "No object is held before placing %s.", object_id.c_str());
        return false;
      }
      if (held_object_ != object_id) {
        RCLCPP_ERROR(get_logger(), "Held object is %s, not %s.", held_object_.c_str(), object_id.c_str());
        return false;
      }
      if (destination_positions_.count(zone_id)) {
        xyz = destination_positions_.at(zone_id);
      } else {
        xyz = get_parameter("scene." + zone_id).as_double_array();
      }
    }

    if (xyz.size() != 3) {
      return false;
    }

    // Keep the gripper closed while the object is temporarily detached from
    // MoveIt. The attached collision object makes the planner unable to find a
    // valid goal when the object is near the destination, even though the
    // physical object is still held by the gripper.
    move_group_->detachObject(object_id);
    planning_scene_.removeCollisionObjects({object_id});

    auto approach = xyz;
    approach[2] += kApproachHeight;
    auto placement_xyz = xyz;
    placement_xyz[2] -= kContactInsetZ;
    if (!move_to_pose(target_pose(approach), "approach " + zone_id) ||
      !move_to_pose(target_pose(placement_xyz), "place " + object_id + " in " + zone_id))
    {
      command_gripper(0.0);
      return false;
    }

    const auto size = get_parameter("scene.object_size").as_double_array();
    if (size.size() != 3 ||
        !planning_scene_.applyCollisionObject(make_box(object_id, xyz, size)) ||
        !move_group_->attachObject(object_id, get_parameter("tool_link").as_string(), kTouchLinks))
    {
      command_gripper(0.0);
      return false;
    }
    if (!command_gripper(0.0)) {
      return false;
    }

    // At the release pose the fingers intentionally overlap the just-placed
    // cube. Remove its world collision object before planning the retreat;
    // otherwise MoveIt rejects the current state as an immediate collision.
    planning_scene_.removeCollisionObjects({object_id});
    move_group_->detachObject(object_id);
    const bool retreated = move_to_pose(target_pose(approach), "retreat " + zone_id);
    if (size.size() == 3) {
      planning_scene_.applyCollisionObject(make_box(object_id, xyz, size));
    }

    // Update object position dynamically after placement
    {
      std::lock_guard<std::mutex> lock(positions_mutex_);
      object_positions_[object_id] = xyz;
      held_object_.clear();
    }

    return retreated;
  }

  void publish_status(const std::string & value)
  {
    std_msgs::msg::String message;
    message.data = value;
    status_publisher_->publish(message);
  }

  void execute_plan(const std_msgs::msg::String::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    is_executing_plan_.store(true);
    
    // Use an RAII wrapper or simple struct to ensure is_executing_plan_ resets safely
    struct ExecGuard {
      std::atomic<bool>& flag;
      ~ExecGuard() { flag.store(false); }
    } guard{is_executing_plan_};

    std::vector<Step> steps;
    std::string reason;
    if (!decode_plan(message->data, steps, reason)) {
      RCLCPP_ERROR(get_logger(), "Rejected incoming plan: %s", reason.c_str());
      publish_status("PLAN_REJECTED " + reason);
      return;
    }

    RCLCPP_INFO(get_logger(), "EXECUTION:");
    for (const auto & step : steps) {
      const auto description = step.skill == "home" ? "home()" :
        step.skill == "pick" ? "pick(" + step.object + ")" :
        "place(" + step.object + ", " + step.zone + ")";
      RCLCPP_INFO(get_logger(), "%s ........ RUNNING", description.c_str());
      publish_status(description + " RUNNING");
      bool success = false;
      if (step.skill == "home") {
        success = move_home();
      } else if (step.skill == "pick") {
        success = pick(step.object);
      } else {
        success = place(step.object, step.zone);
      }
      const std::string result = description + (success ? " SUCCESS" : " FAILED");
      RCLCPP_INFO(get_logger(), "%s", result.c_str());
      publish_status(result);
      if (!success) {
        RCLCPP_ERROR(get_logger(), "TASK FAILED; stopping the remaining skills.");
        publish_status("TASK FAILED");
        return;
      }
    }
    RCLCPP_INFO(get_logger(), "TASK SUCCESS");
    publish_status("TASK SUCCESS");
  }

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_client_;
  rclcpp::CallbackGroup::SharedPtr gripper_callback_group_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr plan_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scene_state_subscription_;
  
  std::map<std::string, std::vector<double>> object_positions_;
  std::map<std::string, std::vector<double>> destination_positions_;
  std::string held_object_;
  std::mutex positions_mutex_;
  std::atomic<bool> is_executing_plan_{false};
  std::mutex execution_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotSkills>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  const bool initialized = node->initialize();
  if (initialized) {
    RCLCPP_INFO(node->get_logger(), "Validated MoveIt skills are ready.");
  }
  if (initialized) {
    spinner.join();
  } else {
    executor.cancel();
    spinner.join();
  }
  rclcpp::shutdown();
  return initialized ? 0 : 1;
}
