#include "my_behavior_tree_pkg/calculategoal_btnode.hpp"
#include <geometry_msgs/msg/point.hpp>

BT::PortsList CalculateGoal::providedPorts()
{
  return { BT::OutputPort<geometry_msgs::msg::Point>("goal") };
}

BT::NodeStatus CalculateGoal::tick()
{
  geometry_msgs::msg::Point mygoal;
  mygoal.x = 1.1;
  mygoal.y = 2.3;
  setOutput<geometry_msgs::msg::Point>("goal", mygoal);

  return BT::NodeStatus::SUCCESS;
}
