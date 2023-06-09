#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

#include "action_interfaces/action/fibonacci.hpp"

#include "my_behavior_tree_pkg/fibonacci_btnode.hpp" // ROS 2 action client BT wrapper
#include "my_behavior_tree_pkg/saysomething_btnode.hpp" // simple non-ROS BT node
#include "my_behavior_tree_pkg/calculategoal_btnode.hpp" // write custom port type example
#include "my_behavior_tree_pkg/printtarget_btnode.hpp" // read from custom port type example
#include "my_behavior_tree_pkg/subscriber_int_btnode.hpp" // subscribe to ROS topic and write value to Blackbord


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



int main(int argc, char * argv[])
{
  // Initialize node
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("bt_task_manager");

  // Get parameters from the launch file
  nh->declare_parameter("path_to_tree", "main_tree.xml");
  std::string path_to_tree = nh->get_parameter("path_to_tree").get_parameter_value().get<std::string>();

  // Define the ROS params for each class that inherits from the RosActionNode class
  BT::RosNodeParams fibonacci_params;
  fibonacci_params.nh = nh;
  fibonacci_params.default_port_value = "fibonacci"; // this is the name of the ROS 2 action server

  BT::RosNodeParams subscriber_int_params;
  subscriber_int_params.nh = nh;
  subscriber_int_params.default_port_value = "/fibonacci/order"; // this is the name of topic subscription

  // Register the nodes into the BT factory
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SaySomething>("SaySomething");
  factory.registerNodeType<CalculateGoal>("CalculateGoal");
  factory.registerNodeType<PrintTarget>("PrintTarget");
  factory.registerNodeType<FibonacciAction>("FibonacciAction", fibonacci_params);
  factory.registerNodeType<SubscriberInt>("SubscriberInt", subscriber_int_params);

  // Create the tree
  // You can load in the tree either from txt or from a file
  // BT::Tree tree = factory.createTreeFromText(xml_text);
  BT::Tree tree = factory.createTreeFromFile(path_to_tree);

  tree.tickWhileRunning();

  return 0;
}
