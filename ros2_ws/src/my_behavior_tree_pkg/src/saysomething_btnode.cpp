#include "my_behavior_tree_pkg/saysomething_btnode.hpp"

BT::PortsList SaySomething::providedPorts()
{
  return { BT::InputPort<std::string>("message") };
}

BT::NodeStatus SaySomething::tick()
{
  std::string message;
  getInput("message", message);
  std::cout << "===========" << std::endl;
  std::cout << "Robot says: " << message << std::endl;
  std::cout << "===========" << std::endl;
  return BT::NodeStatus::SUCCESS;
}
