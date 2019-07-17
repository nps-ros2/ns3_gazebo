cmake_minimum_required(VERSION 3.5)

project(cpp_testbed_runner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)

add_library(testbed_robot src/testbed_robot.cpp)
ament_target_dependencies(testbed_robot
    "example_interfaces"
    "rclcpp"
    "rcutils"
    "std_msgs")

add_library(testbed_robot_callbacks src/testbed_robot_callbacks.cpp)
ament_target_dependencies(testbed_robot_callbacks
    "example_interfaces"
    "rclcpp"
    "rcutils"
    "std_msgs")

add_executable(testbed_runner src/testbed_runner.cpp)

ament_target_dependencies(testbed_runner
  "example_interfaces"
  "rclcpp"
  "rcutils"
  "std_msgs")

target_link_libraries(testbed_runner testbed_robot testbed_robot_callbacks)

install(TARGETS testbed_runner DESTINATION lib/${PROJECT_NAME})


#function(custom_executable target)
#  add_executable(${target} src/${target}.cpp)
#  ament_target_dependencies(${target}
#    "example_interfaces"
#    "rclcpp"
#    "rcutils"
#    "std_msgs")
#  install(TARGETS ${target}
#  DESTINATION lib/${PROJECT_NAME})
#endfunction()
#
## Executables
#custom_executable(testbed_runner)

ament_package()
