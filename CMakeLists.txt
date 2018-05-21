project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# -03 enables optimizations for best performance
add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall -O3 -funroll-loops")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")
# set(CMAKE_VERBOSE_MAKEFILE ON)

set(sources src/MPC.cpp src/main.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(MPC ${sources})

target_link_libraries(MPC ipopt z ssl uv uWS)
