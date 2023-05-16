#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_ros2/behavior_tree_ros2.h>
#include <behaviortree_ros2/ros2_action_node.h>

#include "my_behavior_tree/action/MyAction1.hpp"
#include "my_behavior_tree/action/MyAction2.hpp"

#include "rclcpp_action/rclcpp_action.hpp"


class BehaviorTreeEngineNode : public rclcpp::Node
{
public:
  BehaviorTreeEngineNode()
    : Node("behavior_tree_engine")
  {
    this->declare_parameter("path_to_tree", "main_tree.xml");
    std::string path_to_tree = this->get_parameter("path_to_tree").get_parameter_value().get<std::string>();

    // Register the action nodes with the BehaviorTreeFactory
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<my_behavior_tree::MyAction1>("MyAction1");
    factory.registerNodeType<my_behavior_tree::MyAction2>("MyAction2");

    // Load the behavior tree from the config file
    BT::Tree tree = factory.createTreeFromFile(path_to_tree);

    // Create the ROS 2 behavior tree executor
    BT::ROS2BehaviorTreeExecutor executor(node, tree);

    // Run the behavior tree
    executor.run();
  }

// private:

// };



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_unique<BehaviorTreeEngineNode>());
  rclcpp::shutdown();
  return 0;
}
