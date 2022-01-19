#include <cstdio>
#include "rclcpp/rclcpp.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
//#include <shape_msgs/msg/solid_primitive.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <string>
#include <iostream>
#include <fstream>
#include <cstring>

using namespace std;

const std::string WHITESPACE = " \n\r\t\f\v";

string ltrim(const std::string &s) {
  size_t start = s.find_first_not_of(WHITESPACE);
  return (start == string::npos) ? "" : s.substr(start);
}

string rtrim(const std::string &s) {
  size_t end = s.find_last_not_of(WHITESPACE);
  return (end == string::npos) ? "" : s.substr(0, end + 1);
}

string trim(const string &s) {
  return rtrim(ltrim(s));
}

vector<vector<string>> read_csv(string f_name) {
  // File pointer
  ifstream fin(f_name);
  // Open an existing file
  //fin.open(f_name, ios::in);
  if (!fin.is_open()) {
    cout << "Error opening file: " << f_name << endl;
    exit(1);
  }

  // Read the Data from the file
  // as String Vector
  vector<vector<string>> data;
  vector<string> row;
  string line, word, temp;

  while (!fin.eof()) {
    row.clear();

    // read an entire row and
    // store it in a string variable 'line'
    getline(fin, line);

    // used for breaking words
    stringstream s(line);

    // read every column data of a row and
    // store it in a string variable, 'word'
    while (getline(s, word, ',')) {

      // add all the column data
      // of a row to a vector
      string w = trim(word);  // trim off white space
      w.erase(remove(w.begin(), w.end(), '\"'), w.end());  // get rid of double quotes
      row.push_back(w);
    }
    data.push_back(row);
  }
  return data;
}

void add_box_to_collision_environment(moveit::planning_interface::MoveGroupInterface *move_group,
                                      moveit::planning_interface::PlanningSceneInterface *planning_scene_interface,
                                      vector<string> collision_object_data) {
  // collision object data:
  // name   pos_x   pos_y   pos_z   euler_x   euler_y   euler_z   dim_x   dim_y   dim_z
  //   0      1       2       3       4          5         6        7       8       9
  // (all strings)

  // make collision object
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group->getPlanningFrame();
  collision_object.id = collision_object_data[0];  // name of object

  // make box primitive
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = std::stod(collision_object_data[7]);  // setting dimensions
  primitive.dimensions[primitive.BOX_Y] = std::stod(collision_object_data[8]);
  primitive.dimensions[primitive.BOX_Z] = std::stod(collision_object_data[9]);

  // make pose
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = std::stod(collision_object_data[1]);  // setting position
  box_pose.position.y = std::stod(collision_object_data[2]);
  box_pose.position.z = std::stod(collision_object_data[3]);
  double r = 3.14159 * (std::stod(collision_object_data[4])) / 180.0;  // setting orientation (convert to rads)
  double p = 3.14159 * (std::stod(collision_object_data[5])) / 180.0;
  double y = 3.14159 * (std::stod(collision_object_data[6])) / 180.0;
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(r, p, y);
  box_pose.orientation.x = myQuaternion[0];
  box_pose.orientation.y = myQuaternion[1];
  box_pose.orientation.z = myQuaternion[2];
  box_pose.orientation.w = myQuaternion[3];

  // add box to collision object
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // make vector of the one collision object
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  planning_scene_interface->addCollisionObjects(collision_objects);
}

int main(int argc, char **argv) {
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

  // make sure to set parameter env_csv_file_path when launching this node
  string env_csv_file_path;
  move_group_node->get_parameter("env_csv", env_csv_file_path);
  vector<vector<string>> data = read_csv(env_csv_file_path);
  vector<string> collision_obj;
  for (unsigned int i = 1; i < data.size(); i++) {  // first row (row 0) is just headers so we can skip those
    collision_obj.clear();
    for (unsigned int j = 0; j < data[i].size(); j++) {
      collision_obj[j] = data[i][j];
      add_box_to_collision_environment(&move_group, &planning_scene_interface, collision_obj);
    }
  }

  rclcpp::shutdown();
  return 0;
}