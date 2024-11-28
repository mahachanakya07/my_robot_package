cmake_minimum_required(VERSION 3.5)
project(my_robot_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(realsense2_camera REQUIRED)
find_package(xacro REQUIRED)
find_package(rviz2 REQUIRED)
install(DIRECTORY urdf launch worlds
        DESTINATION share/${PROJECT_NAME})
# Add a target if there is any node you want to compile
# Example of a C++ node (comment out if you don't have one):
# add_executable(robot_node src/robot_node.cpp)
# ament_target_dependencies(robot_node rclcpp std_msgs gazebo_ros)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
