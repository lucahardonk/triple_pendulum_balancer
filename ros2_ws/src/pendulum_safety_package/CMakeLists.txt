cmake_minimum_required(VERSION 3.10)
project(pendulum_safety_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(overrun_node src/overrun_node.cpp)
target_compile_features(overrun_node PUBLIC cxx_std_17)
target_link_libraries(overrun_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  nlohmann_json::nlohmann_json
)

install(TARGETS overrun_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
