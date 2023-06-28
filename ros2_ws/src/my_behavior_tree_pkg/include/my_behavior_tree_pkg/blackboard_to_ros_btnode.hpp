#include <behaviortree_ros2/bt_topic_pub_node.hpp>

// #include <std_msgs/msg/int8.hpp>

template<typename T>
class BlackboardToRos : public BT::RosTopicPubNode<T>
{
public:
  BlackboardToRos(const std::string& name,
                  const BT::NodeConfig& conf,
                  const BT::RosNodeParams& params)
    : BT::RosTopicPubNode<T>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    BT::PortsList list;
    list.insert({BT::InputPort<T>("value")});
    return BT::RosTopicPubNode<T>::providedBasicPorts(list);
  }

  virtual bool setMessage(T& msg)
  {
    BT::TreeNode::getInput("value", msg);
    return true;
  }
};
