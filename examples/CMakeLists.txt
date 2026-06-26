cmake_minimum_required(VERSION 3.14)
project(comotion_examples LANGUAGES CXX)

if(NOT TARGET comotion::comotion)
  find_package(comotion CONFIG REQUIRED)
endif()

add_executable(comotion_library_smoke library_smoke.cpp)
target_link_libraries(comotion_library_smoke PRIVATE comotion::comotion)
