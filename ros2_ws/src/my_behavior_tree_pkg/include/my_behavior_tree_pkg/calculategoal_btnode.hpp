#include <behaviortree_cpp/behavior_tree.h>

class CalculateGoal : public BT::SyncActionNode
{
public:
  CalculateGoal(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};
