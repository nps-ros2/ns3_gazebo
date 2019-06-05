cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include_directories(src ~/repos/ns-3-allinone/ns-3.29/build)
link_directories(~/repos/ns-3-allinone/ns-3.29/build/lib)
list(APPEND CMAKE_CXX_FLAGS)

add_executable(ns3_mobility ns3_mobility.cpp)
target_link_libraries(ns3_mobility
  ns3.29-core-debug
  ns3.29-network-debug
  ns3.29-internet-debug
  ns3.29-wifi-debug
  ns3.29-mobility-debug
  ns3.29-tap-bridge-debug
)

add_executable(ns3_test ns3_test.cpp)
target_link_libraries(ns3_test
  ns3.29-core-debug
  ns3.29-network-debug
  ns3.29-internet-debug
  ns3.29-wifi-debug
  ns3.29-mobility-debug
  ns3.29-tap-bridge-debug
)

