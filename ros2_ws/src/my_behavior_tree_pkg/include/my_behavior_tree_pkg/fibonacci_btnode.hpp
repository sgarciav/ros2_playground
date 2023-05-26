#include <behaviortree_ros2/bt_action_node.hpp>

#include "action_interfaces/action/fibonacci.hpp"

// This example was taken from:
// https://www.behaviortree.dev/docs/ros2_integration/

class FibonacciAction : public BT::RosActionNode<action_interfaces::action::Fibonacci>
{
public:
  FibonacciAction(const std::string& name,
                          const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params)
    : BT::RosActionNode<action_interfaces::action::Fibonacci>(name, conf, params)
  {}

  static BT::PortsList providedPorts();
  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);
};
