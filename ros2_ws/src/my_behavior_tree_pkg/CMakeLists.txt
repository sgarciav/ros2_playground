cmake_minimum_required(VERSION 3.8)
project(my_behavior_tree_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)


add_executable(behavior_tree_engine
  src/behavior_tree_engine.cpp)
target_include_directories(behavior_tree_engine
  PUBLIC include)
ament_target_dependencies(behavior_tree_engine
  rclcpp
  rclcpp_action
  action_interfaces
  # behaviortree_cpp
  behaviortree_ros2
  )

add_library(fibonacci_client_bt
  src/behavior_tree_engine.cpp)
target_include_directories(fibonacci_client_bt
  PUBLIC include)
ament_target_dependencies(fibonacci_client_bt
  rclcpp
  action_interfaces
  # behaviortree_cpp
  behaviortree_ros2
  )


install(TARGETS
  behavior_tree_engine
  fibonacci_client_bt
  DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY launch behavior_trees
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
