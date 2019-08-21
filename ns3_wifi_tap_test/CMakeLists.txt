cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_BUILD_TYPE Debug)

include_directories(src ~/repos/ns-3-allinone/ns-3.29/build)
link_directories(~/repos/ns-3-allinone/ns-3.29/build/lib)
list(APPEND CMAKE_CXX_FLAGS)

add_executable(ns3_wifi_tap_test ns3_wifi_tap_test.cpp)
target_link_libraries(ns3_wifi_tap_test 
  ns3.29-core-debug
  ns3.29-network-debug
  ns3.29-internet-debug
  ns3.29-wifi-debug
  ns3.29-mobility-debug
  ns3.29-tap-bridge-debug
  ns3.29-csma-debug # if you want to see csma work
)

