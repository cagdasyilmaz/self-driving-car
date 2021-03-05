cmake_minimum_required(VERSION 3.5)
project(active_steering_mpc)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++14)

set(CMAKE_CXX_STANDARD 14)

#set(CXX_FLAGS "-Wall")
#set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
#set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")


set(sources src/main.cpp src/KinematicsDynamicsCalculations.h
        src/KinematicsDynamicsCalculations.cpp src/TireCalculations.h src/TireCalculations.cpp
        src/MPC.cpp src/ReferenceGenerator.h src/ReferenceGenerator.cpp)

include_directories(/usr/local/include)
include_directories(/usr/include/python2.7)
include_directories(src/Eigen-3.3)
link_directories(/usr/local/lib)
link_directories(/usr/bin)

add_executable(active_steering_mpc ${sources})

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_link_libraries(active_steering_mpc ipopt python2.7)
else()
    target_link_libraries(active_steering_mpc ipopt python2.7)
endif()
