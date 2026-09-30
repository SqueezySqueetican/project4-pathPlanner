cmake_minimum_required(VERSION 3.10)
project(GridRobotController)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB SOURCES "src/*.cpp")

file(GLOB HEADERS "include/*.hpp")

add_executable(grid_robot ${SOURCES} ${HEADERS})

target_include_directories(grid_robot PRIVATE include)