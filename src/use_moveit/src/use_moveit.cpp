#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv); 
  auto node = std::make_shared<rclcpp::Node>( "use_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("use_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface; 
  MoveGroupInterface move_group_interface(node, "panda_arm");
  // Wait until move_group servers are ready 
  //move_group_interface.waitForServers();

  // Set a target Pose
  geometry_msgs::msg::Pose target_pose; 
  target_pose.orientation.w = 1.0; 
  // identity quaternion 
  target_pose.position.x = 0.5; 
  target_pose.position.y = 0.3; 
  target_pose.position.z = 0.8; 
  move_group_interface.setPoseTarget(target_pose);


  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}