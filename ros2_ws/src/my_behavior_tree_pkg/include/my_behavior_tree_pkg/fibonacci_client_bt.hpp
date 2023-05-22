#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

#include "action_interfaces/action/fibonacci.hpp"

using namespace BT;

class FibonacciActionClientBT : public BT::RosActionNode<action_interfaces::action::Fibonacci>
{
public:
  // FibonacciActionClientBT(const std::string& name,
  //                         const BT::NodeConfig& conf,
  //                         const BT::RosNodeParams& params)
  //   : BT::RosActionNode<action_interfaces::action::Fibonacci>(name, conf, params)
  // {}

  FibonacciActionClientBT(const std::string& name,
                          const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();
  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);
};

// Plugin registration.
// The class FibonacciActionClientBT will self register with name "Fibonacci".
CreateRosNodePlugin(FibonacciActionClientBT, "Fibonacci");
