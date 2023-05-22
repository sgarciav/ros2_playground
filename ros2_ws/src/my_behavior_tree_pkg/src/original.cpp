#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

#include "action_interfaces/action/fibonacci.hpp"

using namespace BT;

class FibonacciActionClientBT : public BT::RosActionNode<action_interfaces::action::Fibonacci>
{
public:
  FibonacciActionClientBT(const std::string& name,
                          const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params)
    : BT::RosActionNode<action_interfaces::action::Fibonacci>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using BT::RosActionNode::providedBasicPorts()
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<unsigned>("order")});
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  // bool setGoal(BT::RosActionNode::Goal& goal) override
  bool setGoal(Goal& goal) override
  {
    // get "order" from the Input port
    getInput("order", goal.order);
    // return true, if we were able to set the goal correctly.
    return true;
  }

  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : wr.result->sequence)
    {
      ss << number << " ";
    }
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    return BT::NodeStatus::SUCCESS;
  }

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
    return BT::NodeStatus::FAILURE;
  }

  // we also support a callback for the feedback, as in
  // the original tutorial.
  // Usually, this callback should return RUNNING, but you
  // might decide, based on the value of the feedback, to abort
  // the action, and consider the TreeNode completed.
  // In that case, return SUCCESS or FAILURE.
  // The Cancel request will be send automatically to the server.
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence)
    {
      ss << number << " ";
    }
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    return BT::NodeStatus::RUNNING;
  }

// private:
//   bool setGoal(Goal& goal) override;
//   BT::NodeStatus onResultReceived(const WrappedResult& wr) override;
//   virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
//   BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);

};

// Plugin registration.
// The class FibonacciActionClientBT will self register with name "Fibonacci".
CreateRosNodePlugin(FibonacciActionClientBT, "Fibonacci");
