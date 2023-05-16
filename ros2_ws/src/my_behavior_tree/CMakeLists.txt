cmake_minimum_required(VERSION 3.8)
project(my_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MyAction.action"
  DEPENDENCIES std_msgs geometry_msgs
)


add_executable(behavior_tree_engine
  src/behavior_tree_engine.cpp)
target_include_directories(behavior_tree_engine
  PUBLIC include)
ament_target_dependencies(behavior_tree_engine
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  behaviortree_ros2
  )


install(TARGETS behavior_tree_engine
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

ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  geometry_msgs
  )

ament_package()
