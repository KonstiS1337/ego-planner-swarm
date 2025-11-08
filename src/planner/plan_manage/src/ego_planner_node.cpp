#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <iostream>

#include <ego_planner/ego_replan_fsm.h>

using namespace ego_planner;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ego_planner_node");

  EGOReplanFSM rebo_replan;

  if(rebo_replan.init(node)) rclcpp::spin(node);
  else RCLCPP_ERROR(node->get_logger(),"Failed to init planner successfully -> shutingdown");

  rclcpp::shutdown();

  return 0;
}

