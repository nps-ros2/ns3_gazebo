# https://cmake.org/examples/
cmake_minimum_required(VERSION 3.5)
project(diff_drive_ns3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(src ~/repos/ns-3-allinone/ns-3.29/build)
link_directories(~/repos/ns-3-allinone/ns-3.29/build/lib)

add_executable(diff_drive_ns3 src/diff_drive_ns3.cpp src/diff_drive_robot.cpp)

target_link_libraries(diff_drive_ns3
  ns3.29-core-debug
  ns3.29-network-debug
  ns3.29-internet-debug
  ns3.29-wifi-debug
  ns3.29-mobility-debug
  ns3.29-tap-bridge-debug
)

ament_target_dependencies(diff_drive_ns3
  rclcpp
  urdf
  geometry_msgs
  nav_msgs
)

install(TARGETS
        diff_drive_ns3
        RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
