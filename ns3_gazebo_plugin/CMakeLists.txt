cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(src ~/repos/ns-3-allinone/ns-3.29/build)
link_directories(${GAZEBO_LIBRARY_DIRS})
link_directories(~/repos/ns-3-allinone/ns-3.29/build/lib)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(diff_drive_ns3_gazebo SHARED ns3_gazebo_world.cpp)
target_link_libraries(diff_drive_ns3_gazebo ${GAZEBO_LIBRARIES}
  ns3.29-core-debug
  ns3.29-network-debug
  ns3.29-internet-debug
  ns3.29-wifi-debug
  ns3.29-mobility-debug
  ns3.29-tap-bridge-debug
)

