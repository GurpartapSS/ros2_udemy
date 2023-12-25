cmake_minimum_required(VERSION 3.8)
project(my_pkg_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(cpp_node src/first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)

add_executable(smartPhone src/smartphone.cpp)
ament_target_dependencies(smartPhone rclcpp example_interfaces)

add_executable(add_two_int_server src/add_two_int_servers.cpp)
ament_target_dependencies(add_two_int_server rclcpp example_interfaces)

add_executable(add_two_int_client_simple src/add_two_int_client_simple.cpp)
ament_target_dependencies(add_two_int_client_simple rclcpp example_interfaces)

add_executable(add_two_int_client_oop src/add_two_int_client_oop.cpp)
ament_target_dependencies(add_two_int_client_oop rclcpp example_interfaces)

install(TARGETS
cpp_node
robot_news_station
smartPhone
add_two_int_server
add_two_int_client_simple
add_two_int_client_oop
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
