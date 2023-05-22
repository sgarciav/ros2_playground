#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

#include "my_behavior_tree_pkg/fibonacci_client_bt.hpp"
#include "action_interfaces/action/fibonacci.hpp"


class BehaviorTreeEngineNode : public rclcpp::Node
{
public:
  BehaviorTreeEngineNode() : Node("behavior_tree_engine")
  {
    this->declare_parameter("path_to_tree", "main_tree.xml");
    std::string path_to_tree = this->get_parameter("path_to_tree").get_parameter_value().get<std::string>();

    BT::RosNodeParams params;
    // params.nh = nh;
    params.default_port_value = "fibonacci"; // name of the action server

    // Register the existing ROS 2 action nodes with the BehaviorTreeFactory
    BT::BehaviorTreeFactory factory;
    // factory.registerSimpleAction("fibonacci",
    //                              std::bind(&Ros2ActionNode<action_interfaces::action::Fibonacci>::Tick, _1, node));

    factory.registerNodeType<FibonacciActionClientBT>("Fibonacci", params);

    // Load the behavior tree from the config file
    BT::Tree tree = factory.createTreeFromFile(path_to_tree);

    tree.tickWhileRunning();

    // // Create the ROS 2 behavior tree executor
    // BT::ROS2BehaviorTreeExecutor executor(node, tree);

    // // Run the behavior tree
    // executor.run();
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BehaviorTreeEngineNode>());
  rclcpp::shutdown();
  return 0;
}
