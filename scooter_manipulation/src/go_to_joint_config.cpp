#include <functional>
#include <memory>
#include <thread>

#include "scooter_interfaces/action/go_to_joint_config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// I'm not doing the visibility control thing because this will never be run on Windows ever for any reason

#define PLANNING_GROUP "ur_manipulator"

namespace go_to_joint_config_cpp {
 class GTJCActionServer : public rclcpp::Node, std::enable_shared_from_this<GTJCActionServer> {
  public:
   using GTJC = scooter_interfaces::action::GoToJointConfig;
   using GoalHandleGTJC = rclcpp_action::ServerGoalHandle<GTJC>;

   explicit GTJCActionServer(std::string name,
                             std::shared_ptr<rclcpp::Node> move_group_node,
                             const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
       : Node(name, options) {
     using namespace std::placeholders;

     move_group_node_ = move_group_node;

     // MoveIt stuff
     // move_group_node_ = rclcpp::Node::make_shared("go_to_joint_config", node_options_);
     move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, PLANNING_GROUP);

     // action server stuff
     rclcpp::executors::SingleThreadedExecutor executor;
     executor.add_node(static_cast<std::shared_ptr<Node>>(this));
     std::thread([&executor]() {executor.spin();}).detach();

     this->action_server_ = rclcpp_action::create_server<GTJC>(
       this,
       "go_to_joint_config",
       std::bind(&GTJCActionServer::handle_goal, this, _1, _2),
       std::bind(&GTJCActionServer::handle_cancel, this, _1),
       std::bind(&GTJCActionServer::handle_accepted, this, _1)
     );
   }

   ~GTJCActionServer() override = default;

  private:
   // MoveIt stuff
   rclcpp::NodeOptions node_options_;
   rclcpp::Node::SharedPtr move_group_node_;
   moveit::planning_interface::MoveGroupInterfacePtr move_group_;
   moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

   // action server stuff
   rclcpp_action::Server<GTJC>::SharedPtr action_server_;
//   std::shared_ptr<rclcpp::Node> move_group_node_;

   rclcpp_action::GoalResponse handle_goal(
     const rclcpp_action::GoalUUID& uuid,
     std::shared_ptr<const GTJC::Goal> goal
   ) {
     RCLCPP_INFO(this->get_logger(), "Received goal request");
     (void)uuid;
     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
   }

   rclcpp_action::CancelResponse handle_cancel(
     const std::shared_ptr<GoalHandleGTJC> goal_handle
   ) {
     RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
     (void)goal_handle;
     return rclcpp_action::CancelResponse::ACCEPT;
   }

   void handle_accepted(const std::shared_ptr<GoalHandleGTJC> goal_handle) {
     using namespace std::placeholders;
     std::thread{std::bind(&GTJCActionServer::execute, this, _1), goal_handle}.detach();
   }

   void execute(const std::shared_ptr<GoalHandleGTJC> goal_handle) {
     const auto goal = goal_handle->get_goal();
     RCLCPP_INFO(this->get_logger(), "Going to joint config");

     // std::vector<double> joint_config(goal->position);

     move_group_->setJointValueTarget(goal->position);
   }

 }; // class GTJCActionServer
} // namespace gtjc_action_server

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr move_group_node = rclcpp::Node::make_shared("moveit_move_group_helper_node", node_options);

  rclcpp::spin(std::make_shared<go_to_joint_config_cpp::GTJCActionServer>("go_to_joint_config",
                                                                                   move_group_node,
                                                                                   node_options));
  rclcpp::shutdown();
  return 0;
}