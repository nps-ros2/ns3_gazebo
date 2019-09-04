cmake_minimum_required(VERSION 3.5)

set(CMAKE_BUILD_TYPE Debug)

project(cpp_testbed_runner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
#find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/TestbedMessage.msg"
    DEPENDENCIES std_msgs builtin_interfaces)

add_library(testbed_robot src/testbed_robot.cpp)
ament_target_dependencies(testbed_robot
    "testbed_message"
    "example_interfaces"
    "rclcpp"
    "rcutils"
    "std_msgs")

rosidl_target_interfaces(testbed_robot ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(testbed_robot_callbacks src/testbed_robot_callbacks.cpp)
ament_target_dependencies(testbed_robot_callbacks
    "example_interfaces"
    "rclcpp"
    "rcutils"
    "std_msgs"
    "testbed_message")

rosidl_target_interfaces(testbed_robot_callbacks
                         ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(setup_reader src/setup_reader.cpp)
ament_target_dependencies(setup_reader
    "testbed_message"
    "example_interfaces"
    "rclcpp"
    "rcutils"
    "std_msgs")

add_library(pipe_writer src/pipe_writer.cpp)

add_library(set_nns src/set_nns.cpp)

add_library(shared_simtime ../ns3_simtime_support/shared_simtime.cc)
target_link_libraries(shared_simtime rt)

add_executable(testbed_runner src/testbed_runner.cpp)

include_directories(../ns3_simtime_support)

ament_target_dependencies(testbed_runner
  "example_interfaces"
  "rclcpp"
  "rcutils"
  "std_msgs")

target_link_libraries(testbed_runner testbed_robot testbed_robot_callbacks
                      setup_reader pipe_writer set_nns shared_simtime)

install(TARGETS testbed_runner DESTINATION lib/${PROJECT_NAME})

file(COPY ../../ns3_testbed/csv_setup/example1.csv DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

ament_package()
