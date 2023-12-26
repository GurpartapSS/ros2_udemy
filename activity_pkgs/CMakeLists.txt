cmake_minimum_required(VERSION 3.8)
project(activity_pkgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(activity_pkgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetLed.srv"
)

add_executable(a2_numberPub src/a02_number_pub.cpp)
ament_target_dependencies(a2_numberPub rclcpp example_interfaces)

add_executable(a2_numberCounter src/a02_number_counter.cpp)
ament_target_dependencies(a2_numberCounter rclcpp example_interfaces)

add_executable(a3_numberReset src/a03_numberCounterReset.cpp)
ament_target_dependencies(a3_numberReset rclcpp example_interfaces)

add_executable(a4_batteryStatePub src/a04_batteryState_pub.cpp)
ament_target_dependencies(a4_batteryStatePub rclcpp activity_pkgs)

install(TARGETS
a2_numberPub
a2_numberCounter
a3_numberReset
a4_batteryStatePub
DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
