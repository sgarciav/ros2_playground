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
    // this->declare_parameter("path_to_tree", "main_tree.xml");
    // std::string path_to_tree = this->get_parameter("path_to_tree").get_parameter_value().get<std::string>();

    // debug (so we can run the node directly without the launch file)
    std::string path_to_tree = "/home/ros/workspaces/ros2_ws/install/my_behavior_tree_pkg/share/my_behavior_tree_pkg/behavior_trees/main_tree.xml";

    BT::RosNodeParams params;
    // params.nh = nh;
    params.default_port_value = "fibonacci";
    // params.action_name = "fibonacci";
    // params.action_server_name = "fibonacci";

    // debug
    RCLCPP_INFO(this->get_logger(),
                "========================== registering node");

    // Register the existing ROS 2 action nodes with the BehaviorTreeFactory
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<FibonacciActionClientBT>("Fibonacci", params);

    // debug
    RCLCPP_INFO(this->get_logger(),
                "========================== path to tree: %s", path_to_tree.c_str());

    // Load the behavior tree from the config file
    BT::Tree tree = factory.createTreeFromFile(path_to_tree);

    // debug
    RCLCPP_INFO(this->get_logger(),
                "========================== about to run");

    tree.tickWhileRunning();


    // debug
    RCLCPP_INFO(this->get_logger(),
                "========================== should run");

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
