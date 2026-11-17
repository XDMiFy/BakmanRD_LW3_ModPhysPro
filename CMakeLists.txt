cmake_minimum_required(VERSION 3.16)
project(ModPhysPro CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)  # замеры времени — только в Release
endif ()

find_package(SDL2 REQUIRED)
add_executable(ModPhysPro main.cpp)
target_link_libraries(ModPhysPro PRIVATE SDL2::SDL2)
