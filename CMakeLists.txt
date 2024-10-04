cmake_minimum_required(VERSION 3.8)
project(ros2_reflect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

if(BUILD_TESTING)
  include(test/CMakeLists.txt)
endif()

ament_package()
