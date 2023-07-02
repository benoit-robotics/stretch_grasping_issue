#include <functional>
#include <memory>
#include <thread>

#include <tf2_eigen/tf2_eigen.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("grasp");

Eigen::Isometry3d getPose(const std::string &goal_frame,
                          const std::string &target_frame,
                          std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
{
  Eigen::Isometry3d goal_pose_fixed_frame;
  geometry_msgs::msg::TransformStamped t = tf_buffer->lookupTransform(target_frame, goal_frame, tf2::TimePointZero);
  goal_pose_fixed_frame = tf2::transformToEigen(t.transform);
  return goal_pose_fixed_frame;
}

Eigen::Isometry3d getPoseInOdom(const Eigen::Isometry3d &t_base,
                                std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
{
  // transform pose in base_link to odom
  Eigen::Isometry3d t_base_odom = getPose("base_link", "odom", tf_buffer);
  Eigen::Isometry3d t_odom;
  t_odom = t_base_odom * t_base;
  return t_odom;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr move_group_node = rclcpp::Node::make_shared("movegroup", node_options);

  static const std::string PLANNING_GROUP_BASE_ARM = "mobile_base_arm";
  static const std::string PLANNING_GROUP_ARM = "stretch_arm";

  auto move_group_interface_robot = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, PLANNING_GROUP_BASE_ARM);
  move_group_interface_robot->setMaxVelocityScalingFactor(1.0);
  move_group_interface_robot->setMaxAccelerationScalingFactor(1.0);
  move_group_interface_robot->setNumPlanningAttempts(10);
  move_group_interface_robot->setWorkspace(-200., -200., -200., 200., 200., 200.);

  auto move_group_interface_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, PLANNING_GROUP_ARM);
  move_group_interface_arm->setMaxVelocityScalingFactor(1.0);
  move_group_interface_arm->setMaxAccelerationScalingFactor(1.0);
  move_group_interface_arm->setNumPlanningAttempts(10);
  move_group_interface_arm->setWorkspace(-200., -200., -200., 200., 200., 200.);

  ///
  std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>(move_group_node->get_clock());
  std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]()
              { executor.spin(); })
      .detach();

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_interface_robot->getPlanningFrame().c_str());
  std::string planning_frame = move_group_interface_robot->getPlanningFrame();

  // We can also print the name of the end-effector link for this group
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_interface_robot->getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "Reference frame: %s", move_group_interface_robot->getPoseReferenceFrame().c_str());
  std::string ee_frame = move_group_interface_robot->getEndEffectorLink();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Beginning

  // This code works, group ==  stretch_arm
  RCLCPP_INFO(LOGGER, "Moving the arm 3 times with the stretch_arm group");
  std::vector<double> joint_group_pos;
  moveit::core::RobotStatePtr current_state;
  current_state = move_group_interface_arm->getCurrentState(10);
  auto joint_model_group = move_group_interface_arm->getCurrentState()->getJointModelGroup("stretch_arm");
  current_state->copyJointGroupPositions(joint_model_group, joint_group_pos);
  joint_group_pos[0] += 0.1;
  move_group_interface_arm->setJointValueTarget(joint_group_pos);
  move_group_interface_arm->move();

  // moving the arm back to the previous position
  current_state = move_group_interface_arm->getCurrentState(10);
  Eigen::Isometry3d target_pose = current_state->getGlobalLinkTransform(ee_frame); // pose in base_link
  joint_model_group = move_group_interface_arm->getCurrentState()->getJointModelGroup("stretch_arm");
  current_state->copyJointGroupPositions(joint_model_group, joint_group_pos);
  joint_group_pos[0] -= 0.1;
  move_group_interface_arm->setJointValueTarget(joint_group_pos);
  move_group_interface_arm->move(); 

  // moving the arm again, to the first position
  Eigen::Isometry3d target_pose_odom = getPoseInOdom(target_pose, tf_buffer);
  move_group_interface_arm->setPoseTarget(target_pose_odom);
  move_group_interface_arm->move(); // moving again to the first position

  // Repeating the same thing, but with a different group
  // This code doesn't work, group == mobile_base_arm
  RCLCPP_INFO(LOGGER, "Moving the arm 3 times with the mobile_base_arm group");
  current_state = move_group_interface_robot->getCurrentState(10);
  joint_model_group = move_group_interface_robot->getCurrentState()->getJointModelGroup("mobile_base_arm");
  current_state->copyJointGroupPositions(joint_model_group, joint_group_pos);
  joint_group_pos[3] += 0.1;
  move_group_interface_robot->setJointValueTarget(joint_group_pos);
  move_group_interface_robot->move();

  current_state = move_group_interface_robot->getCurrentState(10);
  target_pose = current_state->getGlobalLinkTransform(ee_frame);
  joint_model_group = move_group_interface_robot->getCurrentState()->getJointModelGroup("mobile_base_arm");
  current_state->copyJointGroupPositions(joint_model_group, joint_group_pos);
  joint_group_pos[3] -= 0.1;
  move_group_interface_robot->setJointValueTarget(joint_group_pos);
  move_group_interface_robot->move(); // moving back

  RCLCPP_INFO(LOGGER, "This last call fails");
  target_pose_odom = getPoseInOdom(target_pose, tf_buffer);
  move_group_interface_robot->setPoseTarget(target_pose_odom);
  move_group_interface_robot->move(); // failure

  return 0;
}
