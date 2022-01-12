#include <cstdio>
#include "rclcpp/rclcpp.hpp"


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/collision_object.hpp>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world scooter_manipulation package\n");
  return 0;
}
