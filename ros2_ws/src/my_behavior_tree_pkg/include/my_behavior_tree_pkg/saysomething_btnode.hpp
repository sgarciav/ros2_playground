#include <behaviortree_cpp/behavior_tree.h>

class SaySomething : public BT::SyncActionNode
{
public:
  SaySomething(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};
