cmake_minimum_required(VERSION 3.20)
project(encoders_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)
find_package(builtin_interfaces REQUIRED)


find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Encoder.msg"
  "msg/EncoderState.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(encoders_driver src/encoders_driver.cpp)

target_compile_features(encoders_driver PRIVATE cxx_std_17)

# Collega il nodo ai messaggi custom generati nello stesso package.
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)

target_link_libraries(encoders_driver
  rclcpp::rclcpp
  ${std_srvs_TARGETS}
  "${cpp_typesupport_target}"
  Threads::Threads
)

install(TARGETS encoders_driver
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
