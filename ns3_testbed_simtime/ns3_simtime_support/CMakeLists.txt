cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_BUILD_TYPE Debug)

include_directories(src ~/repos/ns-3-allinone/ns-3.29/build)
link_directories(~/repos/ns-3-allinone/ns-3.29/build/lib)
list(APPEND CMAKE_CXX_FLAGS)

# simtime support
add_library(shared_simtime shared_simtime.cc)
add_executable(shared_simetime_test shared_simtime_test.cc)
target_link_libraries(shared_simetime_test shared_simtime rt)

