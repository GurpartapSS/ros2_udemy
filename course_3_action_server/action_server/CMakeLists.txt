cmake_minimum_required(VERSION 3.8)
project(udemy_actionserver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(udemy_actionserver REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CountUntil.action"
)

add_executable(count_until_server src/count_until_server.cpp)
ament_target_dependencies(count_until_server rclcpp rclcpp_action udemy_actionserver)
add_executable(count_until_queue_goals_server src/count_until_queue_goals_server.cpp)
ament_target_dependencies(count_until_queue_goals_server rclcpp rclcpp_action udemy_actionserver)
add_executable(count_until_client src/count_until_client.cpp)
ament_target_dependencies(count_until_client rclcpp rclcpp_action udemy_actionserver)

install(TARGETS
  count_until_server
  count_until_client
  count_until_queue_goals_server
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
