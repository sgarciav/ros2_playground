#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

#include "my_behavior_tree_pkg/fibonacci_btnode.hpp" // ROS 2 action client BT wrapper
#include "my_behavior_tree_pkg/saysomething_btnode.hpp" // simple non-ROS BT node
#include "my_behavior_tree_pkg/calculategoal_btnode.hpp" // write custom port type example
#include "my_behavior_tree_pkg/printtarget_btnode.hpp" // read from custom port type example
#include "my_behavior_tree_pkg/ros_to_blackboard_btnode.hpp" // subscribe to ROS topic and write value to Blackbord
#include "my_behavior_tree_pkg/blackboard_to_ros_btnode.hpp" // read value from Blackboard and publish to ROS topic

#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>

// static const char* xml_text = R"(
// <root BTCPP_format="4" main_tree_to_execute="MainTree">
//   <BehaviorTree ID="MainTree">
//     <Sequence name="main_sequence">
//       <SaySomething message="Hello, World!" />
//       <FibonacciAction order="5" />
//     </Sequence>
//   </BehaviorTree>
// </root>
// )";

BT::RosNodeParams set_params(std::shared_ptr<rclcpp::Node> nh, std::string name)
{
  BT::RosNodeParams params;
  params.nh = nh;
  params.default_port_value = name; // server name (for actions) or topic name (for pubs and subs)
  return params;
}


int main(int argc, char * argv[])
{
  // Initialize node
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("bt_task_manager");

  // Get parameters from the launch file
  nh->declare_parameter("path_to_tree", "main_tree.xml");
  std::string path_to_tree = nh->get_parameter("path_to_tree").get_parameter_value().get<std::string>();

  // Register the nodes into the BT factory
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SaySomething>("SaySomething");
  factory.registerNodeType<CalculateGoal>("CalculateGoal");
  factory.registerNodeType<PrintTarget>("PrintTarget");
  factory.registerNodeType<FibonacciAction>("FibonacciAction", set_params(nh, "fibonacci"));
  // note that these name have to be unique
  factory.registerNodeType<RosToBlackboard<std_msgs::msg::Int8>>("RosToBlackboardInt", set_params(nh, "/ros2bb/int"));
  factory.registerNodeType<BlackboardToRos<std_msgs::msg::String>>("BlackboardToRosString", set_params(nh, "/bb2ros/string"));
  factory.registerNodeType<BlackboardToRos<geometry_msgs::msg::Point>>("BlackboardToRosPoint", set_params(nh, "/bb2ros/point"));

  // Create the tree
  // You can load in the tree either from txt or from a file
  // BT::Tree tree = factory.createTreeFromText(xml_text);
  BT::Tree tree = factory.createTreeFromFile(path_to_tree);

  std::cout << " ----- Ticking Tree ----- " << std::endl;
  tree.tickWhileRunning();

  return 0;
}
