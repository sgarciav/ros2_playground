#include <behaviortree_ros2/bt_topic_sub_node.hpp>

#include <std_msgs/msg/int32.hpp>


class SubscriberInt : public BT::RosTopicSubNode<std_msgs::msg::Int32>
{
public:
  SubscriberInt(const std::string& name,
                const BT::NodeConfig& conf,
                const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<std_msgs::msg::Int32>(name, conf, params)
  {}

  static BT::PortsList providedPorts();
  virtual BT::NodeStatus onTick(const typename std_msgs::msg::Int32::SharedPtr& last_msg);
};
