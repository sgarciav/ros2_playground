cmake_minimum_required(VERSION 3.8)
project(my_action_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(fibonacci_server SHARED
  src/fibonacci_action_server.cpp)
target_include_directories(fibonacci_server
  PUBLIC include)
# target_include_directories(fibonacci_server PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
target_compile_definitions(fibonacci_server
  PRIVATE "MY_ACTION_PKG_BUILDING_DLL")
ament_target_dependencies(fibonacci_server
  action_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  )
rclcpp_components_register_node(fibonacci_server PLUGIN
  "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE
  fibonacci_action_server
  )



install(TARGETS
  fibonacci_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
# install(TARGETS fibonacci_server
#   DESTINATION lib/${PROJECT_NAME}
#   )


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
