#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

#include <behaviortree_ros2/bt_action_node.hpp>

#include "action_interfaces/action/fibonacci.hpp"
#include "my_behavior_tree_pkg/fibonacci_btnode.hpp" // ROS 2 action client BT wrapper


// Example of a simple non-ROS action
class SaySomething : public BT::SyncActionNode
{
public:
  SaySomething(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("message") };
  }

  BT::NodeStatus tick() override
  {
    std::string message;
    getInput("message", message);
    std::cout << "===========" << std::endl;
    std::cout << "Robot says: " << message << std::endl;
    std::cout << "===========" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};



static const char* xml_text = R"(
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="main_sequence">
      <SaySomething message="Hello, World!" />
      <FibonacciAction order="5" />
    </Sequence>
  </BehaviorTree>
</root>
)";




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("bt_task_manager");

  BT::RosNodeParams params;
  params.nh = nh;
  params.default_port_value = "fibonacci";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SaySomething>("SaySomething");
  factory.registerNodeType<FibonacciAction>("FibonacciAction", params);

  auto tree = factory.createTreeFromText(xml_text);

  tree.tickWhileRunning();

  return 0;
}
