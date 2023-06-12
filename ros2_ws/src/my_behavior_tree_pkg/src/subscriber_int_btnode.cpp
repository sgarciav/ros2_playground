#include "my_behavior_tree_pkg/subscriber_int_btnode.hpp"

#include <std_msgs/msg/int32.hpp>

BT::PortsList SubscriberInt::providedPorts()
{
  BT::PortsList list;
  list.insert({BT::OutputPort<int>("order")});
  return providedBasicPorts(list);
}

BT::NodeStatus SubscriberInt::onTick(const typename std_msgs::msg::Int32::SharedPtr& last_msg)
{
  if (last_msg == nullptr)
  {
    RCLCPP_WARN(node_->get_logger(), "msg is null");
    return BT::NodeStatus::FAILURE;
  }

  int order = last_msg->data;
  RCLCPP_INFO(node_->get_logger(), "Tree has been ticked. Heard: %d", order);
  setOutput<int>("order", order);

  return BT::NodeStatus::SUCCESS;
}
