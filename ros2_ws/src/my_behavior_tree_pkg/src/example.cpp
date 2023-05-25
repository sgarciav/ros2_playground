#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

#include <behaviortree_ros2/bt_action_node.hpp>

#include "action_interfaces/action/fibonacci.hpp"

using namespace BT;

class SaySomething : public SyncActionNode
{
public:
  SaySomething(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
  {}

  static PortsList providedPorts()
  {
    return { InputPort<std::string>("message") };
  }

  NodeStatus tick() override
  {
    std::string message;
    getInput("message", message);
    std::cout << "===========" << std::endl;
    std::cout << "Robot says: " << message << std::endl;
    return NodeStatus::SUCCESS;
  }
};

class FibonacciSequence : public RosActionNode<action_interfaces::action::Fibonacci>
{
public:
  FibonacciSequence(const std::string& name,
                    const NodeConfig& conf,
                    const RosNodeParams& params)
    : RosActionNode<action_interfaces::action::Fibonacci>(name, conf, params)
  {}

  // static PortsList providedPorts()
  // {
  //   return providedBasicPorts({InputPort<unsigned>("order")});
  // }

  static PortsList providedPorts()
  {
    return { InputPort<int>("order") };
  }

  bool setGoal(Goal& goal)
  {
    // get "order" from the Input port
    getInput("order", goal.order);
    // return true, if we were able to set the goal correctly.
    return true;
  }

  NodeStatus onResultReceived(const WrappedResult& wr)
  {
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : wr.result->sequence)
    {
      ss << number << " ";
    }
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    return NodeStatus::SUCCESS;
  }

  NodeStatus onFailure(ActionNodeErrorCode error)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
  }

  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence)
    {
      ss << number << " ";
    }
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    return NodeStatus::RUNNING;
  }

};




static const char* xml_text = R"(
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="main_sequence">
      <SaySomething message="Hello, World!" />
      <FibonacciSequence order="5" />
    </Sequence>
  </BehaviorTree>
</root>
)";

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("example_node");

  RosNodeParams params;
  params.nh = nh;
  params.default_port_value = "fibonacci";

  BehaviorTreeFactory factory;
  factory.registerNodeType<SaySomething>("SaySomething");
  factory.registerNodeType<FibonacciSequence>("FibonacciSequence", params);

  auto tree = factory.createTreeFromText(xml_text);

  tree.tickWhileRunning();

  return 0;
}
