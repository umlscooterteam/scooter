#include <cstdio>
#include "rclcpp/rclcpp.hpp"


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
//#include <shape_msgs/msg/solid_primitive.hpp>

void add_box_to_collision_environment(moveit::planning_interface::MoveGroupInterface* move_group,
                                      moveit::planning_interface::PlanningSceneInterface* planning_scene_interface) {
  // make collision object
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group->getPlanningFrame();
  collision_object.id = "box1";

  // make box primitive
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 0.1;
  primitive.dimensions[primitive.BOX_Z] = 0.1;

  // make pose
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.5;
  box_pose.position.y = 0.5;
  box_pose.position.z = 0.5;
  box_pose.orientation.w = 1.0;

  // add box to collision object
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // make vector of the one collision object
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  planning_scene_interface->addCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  // init ROS stuff
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("static_collision_object_publisher");

  // create executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // init MoveIt stuff
  static const std::string PLANNING_GROUP = "ur_manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  add_box_to_collision_environment(&move_group, &planning_scene_interface);

  rclcpp::shutdown();
  return 0;
}