cmake_minimum_required(VERSION 3.15)
project(UAV_Tracking LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(UAVFramework INTERFACE)
target_include_directories(UAVFramework INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  # FIXED: Changed -03 (zero) to -O3 (Letter O)
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
endif()

add_executable(main_run src/main_UAVTracking.cpp)
target_link_libraries(main_run PRIVATE UAVFramework)

find_package(GTest REQUIRED)
include(GoogleTest)

file(GLOB TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp")

add_executable(run_tests ${TEST_SOURCES})
target_link_libraries(run_tests PRIVATE UAVFramework GTest::gtest_main)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang" AND NOT WIN32)
  target_compile_options(run_tests PRIVATE -fsanitize=address)
  target_link_options(run_tests PRIVATE -fsanitize=address)
endif()

gtest_discover_tests(run_tests)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/plots_UAVTracking.py
  ${CMAKE_CURRENT_BINARY_DIR}/plots_UAVTracking.py
  COPYONLY
)