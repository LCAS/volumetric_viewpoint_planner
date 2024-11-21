cmake_minimum_required(VERSION 3.8)
project(moveit2_commander_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_view REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)

add_executable(pose_commander src/pose_commander.cpp)
target_include_directories(pose_commander PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pose_commander PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
pose_commander  
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "graph_msgs"
  "image_view"
  "moveit_core"
  "moveit_visual_tools"
  "rviz_visual_tools"
)

# Install configuration files
install(DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS pose_commander
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
