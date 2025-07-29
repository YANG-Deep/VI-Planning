cmake_minimum_required(VERSION 3.8)
project(grid_future_cells_publisher_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(viplanning_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(grid_future_cells_publisher src/grid_future_cells_publisher_node.cpp)
ament_target_dependencies(grid_future_cells_publisher rclcpp autoware_auto_perception_msgs nav_msgs viplanning_msgs)

install(TARGETS
  grid_future_cells_publisher
  DESTINATION lib/${PROJECT_NAME})


ament_package()
