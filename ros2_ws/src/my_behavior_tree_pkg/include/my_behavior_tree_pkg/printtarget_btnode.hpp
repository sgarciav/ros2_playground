#include <behaviortree_cpp/behavior_tree.h>

class PrintTarget : public BT::SyncActionNode
{
public:
  PrintTarget(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};
