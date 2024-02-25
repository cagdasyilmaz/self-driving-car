cmake_minimum_required(VERSION 3.5)
project(active_steering_mpc)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)

#set(CMAKE_CXX_COMPILER "/usr/bin/g++-11")
set(CMAKE_CXX_COMPILER "/usr/bin/clang++-12")
#set(CMAKE_CXX_COMPILER "/opt/intel/oneapi/compiler/2023.1.0/linux/bin/icpx")

#set(CXX_FLAGS "-Wall")
#set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math")
#set(CMAKE_CXX_FLAGS_DEBUG "-g")
#add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=0)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

include_directories(/usr/local/include)
include_directories(/usr/include)
link_directories(/usr/lib)
link_directories(/usr/lib64)
link_directories(/usr/local/lib)

include_directories(/usr/include/python3.8/)
include_directories(/usr/local/lib/python3.8/site-packages/)

set(sources src/main.cpp src/KinematicsDynamicsCalculations.h
            src/KinematicsDynamicsCalculations.cpp src/TireCalculations.h src/TireCalculations.cpp
            src/MPC.cpp src/ReferenceGenerator.h src/ReferenceGenerator.cpp)

set(CommonLibs cppad_lib ipopt python3.8)

add_executable(${PROJECT_NAME} ${sources})
target_link_libraries(${PROJECT_NAME} ${CommonLibs})