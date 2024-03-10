cmake_minimum_required(VERSION 3.8)
project(activity_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(activity_pkg REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveRobo.action"
  "msg/CancelMove.msg"
)

add_executable(activity1_action_server src/activity1_action_server.cpp)
ament_target_dependencies(activity1_action_server rclcpp rclcpp_action activity_pkg)

INSTALL(TARGETS
  activity1_action_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
