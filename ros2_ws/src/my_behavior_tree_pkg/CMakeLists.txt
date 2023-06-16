cmake_minimum_required(VERSION 3.8)
project(my_behavior_tree_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies

find_package(ament_cmake REQUIRED)
find_package(action_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# BT Nodes

add_library(subscriber_int_btnode STATIC
  src/subscriber_int_btnode.cpp)
target_include_directories(subscriber_int_btnode
  PUBLIC include)
ament_target_dependencies(subscriber_int_btnode
  rclcpp
  action_interfaces
  behaviortree_cpp
  behaviortree_ros2
  std_msgs
  )

add_library(ros_to_blackboard_btnode STATIC
  src/ros_to_blackboard_btnode.cpp)
target_include_directories(ros_to_blackboard_btnode
  PUBLIC include)
ament_target_dependencies(ros_to_blackboard_btnode
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  std_msgs
  )

add_library(fibonacci_btnode STATIC
  src/fibonacci_btnode.cpp)
target_include_directories(fibonacci_btnode
  PUBLIC include)
ament_target_dependencies(fibonacci_btnode
  rclcpp
  action_interfaces
  behaviortree_cpp
  behaviortree_ros2
  )

add_library(saysomething_btnode STATIC
  src/saysomething_btnode.cpp)
target_include_directories(saysomething_btnode
  PUBLIC include)
ament_target_dependencies(saysomething_btnode
  rclcpp
  behaviortree_cpp
  )

add_library(calculategoal_btnode STATIC
  src/calculategoal_btnode.cpp)
target_include_directories(calculategoal_btnode
  PUBLIC include)
ament_target_dependencies(calculategoal_btnode
  rclcpp
  behaviortree_cpp
  geometry_msgs
  )

add_library(printtarget_btnode STATIC
  src/printtarget_btnode.cpp)
target_include_directories(printtarget_btnode
  PUBLIC include)
ament_target_dependencies(printtarget_btnode
  rclcpp
  behaviortree_cpp
  geometry_msgs
  )

# Task Manager

add_executable(bt_task_manager
  src/bt_task_manager.cpp)
target_include_directories(bt_task_manager
  PUBLIC include)
target_link_libraries(bt_task_manager # link all the BT nodes
  subscriber_int_btnode
  saysomething_btnode
  fibonacci_btnode
  calculategoal_btnode
  printtarget_btnode
  ros_to_blackboard_btnode
  )
ament_target_dependencies(bt_task_manager
  rclcpp
  rclcpp_action
  behaviortree_cpp
  behaviortree_ros2
  std_msgs
  )


# ====================

install(TARGETS
  ros_to_blackboard_btnode
  subscriber_int_btnode
  saysomething_btnode
  fibonacci_btnode
  calculategoal_btnode
  printtarget_btnode
  bt_task_manager
  DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY include launch behavior_trees
  DESTINATION share/${PROJECT_NAME}
  )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
