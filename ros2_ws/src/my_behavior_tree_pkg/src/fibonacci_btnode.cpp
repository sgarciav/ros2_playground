#include "my_behavior_tree_pkg/fibonacci_btnode.hpp"

#include <std_msgs/msg/int8.hpp>

// This example was taken from:
// https://www.behaviortree.dev/docs/ros2_integration/


// The specific ports of this Derived class
// should be merged with the ports of the base class,
// using BT::RosActionNode::providedBasicPorts()
BT::PortsList FibonacciAction::providedPorts()
{
  BT::PortsList list;
  list.insert({BT::InputPort<std_msgs::msg::Int8>("order")});
  return providedBasicPorts(list);
}

// This is called when the TreeNode is ticked and it should
// send the request to the action server
// bool setGoal(BT::RosActionNode::Goal& goal) override
bool FibonacciAction::setGoal(Goal& goal)
{
  // get "order" from the Input port
  std_msgs::msg::Int8 msg_ros;
  getInput("order", msg_ros);
  goal.order = msg_ros.data;
  // return true, if we were able to set the goal correctly.
  return true;
}

// Callback executed when the reply is received.
// Based on the reply you may decide to return SUCCESS or FAILURE.
BT::NodeStatus FibonacciAction::onResultReceived(const WrappedResult& wr)
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
BT::NodeStatus FibonacciAction::onFailure(BT::ActionNodeErrorCode error)
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
BT::NodeStatus FibonacciAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
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
