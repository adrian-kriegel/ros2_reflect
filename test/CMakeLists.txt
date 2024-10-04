
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)

ENABLE_TESTING()

ament_add_gmock(
  ${PROJECT_NAME}_test 
  test/reflect_test.cpp
)

ament_target_dependencies(${PROJECT_NAME}_test rclcpp)
