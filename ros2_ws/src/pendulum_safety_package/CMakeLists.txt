cmake_minimum_required(VERSION 3.10)
project(pendulum_safety_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(odrive_can REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PendulumStatus.msg"
  DEPENDENCIES std_msgs
)

add_executable(overrun_node src/overrun_node.cpp)
target_compile_features(overrun_node PUBLIC cxx_std_17)
target_link_libraries(overrun_node PUBLIC rclcpp::rclcpp ${odrive_can_TARGETS})

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(overrun_node PUBLIC "${cpp_typesupport_target}")

install(TARGETS overrun_node DESTINATION lib/${PROJECT_NAME})

ament_package()