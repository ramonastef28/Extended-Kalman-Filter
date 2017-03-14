project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

include_directories(include)

file(GLOB sources "src/*.cpp")

add_executable(ExtendedKF ${sources})
