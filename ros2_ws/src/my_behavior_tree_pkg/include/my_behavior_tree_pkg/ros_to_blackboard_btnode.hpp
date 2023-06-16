#include <behaviortree_ros2/bt_topic_sub_node.hpp>

#include <std_msgs/msg/int8.hpp>

// template <typename T>

using T = std_msgs::msg::Int8;

class RosToBlackboard : public BT::RosTopicSubNode<T>
{
public:
  RosToBlackboard(const std::string& name,
                  const BT::NodeConfig& conf,
                  const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<T>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    BT::PortsList list;
    list.insert({BT::OutputPort<T>("value")});
    return providedBasicPorts(list);
  }

  virtual BT::NodeStatus onTick(const typename T::SharedPtr& last_msg)
  {
    BT::NodeStatus status = BT::NodeStatus::FAILURE;

    if (last_msg != nullptr)
    {
      setOutput<T>("value", *last_msg);
      status = BT::NodeStatus::SUCCESS;
    }

    return status;
  }
};
