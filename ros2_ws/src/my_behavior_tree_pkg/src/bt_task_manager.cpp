#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

#include "my_behavior_tree_pkg/ros_to_blackboard_btnode.hpp"

#include "my_behavior_tree_pkg/fibonacci_btnode.hpp" // ROS 2 action client BT wrapper
#include "my_behavior_tree_pkg/saysomething_btnode.hpp" // simple non-ROS BT node
#include "my_behavior_tree_pkg/calculategoal_btnode.hpp" // write custom port type example
#include "my_behavior_tree_pkg/printtarget_btnode.hpp" // read from custom port type example
#include "my_behavior_tree_pkg/subscriber_int_btnode.hpp" // subscribe to ROS topic and write value to Blackbord

#include <std_msgs/msg/int8.hpp>

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
  nh->declare_parameter("subscriber_topic_name", "/bt_sub_topic");

  std::string path_to_tree = nh->get_parameter("path_to_tree").get_parameter_value().get<std::string>();
  std::string subscriber_topic_name = nh->get_parameter("subscriber_topic_name").get_parameter_value().get<std::string>();

  // Register the nodes into the BT factory
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SaySomething>("SaySomething");
  factory.registerNodeType<CalculateGoal>("CalculateGoal");
  factory.registerNodeType<PrintTarget>("PrintTarget");
  factory.registerNodeType<FibonacciAction>("FibonacciAction", set_params(nh, "fibonacci"));
  factory.registerNodeType<SubscriberInt>("SubscriberInt", set_params(nh, "/fibonacci/order"));
  factory.registerNodeType<RosToBlackboard>("RosToBlackboard", set_params(nh, subscriber_topic_name));

  // Create the tree
  // You can load in the tree either from txt or from a file
  // BT::Tree tree = factory.createTreeFromText(xml_text);
  BT::Tree tree = factory.createTreeFromFile(path_to_tree);

  tree.tickWhileRunning();

  return 0;
}
