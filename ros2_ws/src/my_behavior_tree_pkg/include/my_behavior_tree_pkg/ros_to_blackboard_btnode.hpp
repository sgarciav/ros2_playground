#include <behaviortree_ros2/bt_topic_sub_node.hpp>

template<typename T>
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
    return BT::RosTopicSubNode<T>::providedBasicPorts(list);
  }

  virtual BT::NodeStatus onTick(const typename T::SharedPtr& last_msg)
  {
    BT::NodeStatus status = BT::NodeStatus::FAILURE;

    if (last_msg != nullptr)
    {
      BT::TreeNode::setOutput<T>("value", *last_msg);
      status = BT::NodeStatus::SUCCESS;
    }

    return status;
  }
};
