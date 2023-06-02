#include "my_behavior_tree_pkg/printtarget_btnode.hpp"
#include <geometry_msgs/msg/point.hpp>

BT::PortsList PrintTarget::providedPorts()
{
  // Optionally, a port can have a human readable description
  const char* description = "Simply print the goal on console...";
  return { BT::InputPort<geometry_msgs::msg::Point>("target", description) };
}

BT::NodeStatus PrintTarget::tick()
{
  auto res = getInput<geometry_msgs::msg::Point>("target");
  if( !res )
  {
    throw BT::RuntimeError("error reading port [target]:", res.error());
  }
  geometry_msgs::msg::Point target = res.value();
  printf("Target positions: [ %.1f, %.1f ]\n", target.x, target.y );
  return BT::NodeStatus::SUCCESS;
}
