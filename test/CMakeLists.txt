cmake_minimum_required(VERSION 3.8)
project(test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(graph_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Specify the executables
add_executable(SensorBridge src/SensorBridge.cpp)
add_executable(arm_control_test src/arm_control_test.cpp)

# Include directories
target_include_directories(SensorBridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(arm_control_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Specify the dependencies for the executables
ament_target_dependencies(SensorBridge
  rclcpp
  moveit_ros_planning
  moveit_ros_move_group
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  geometry_msgs
)

ament_target_dependencies(arm_control_test
  rclcpp
  moveit_ros_planning
  moveit_ros_move_group
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  geometry_msgs
  graph_msgs
)

# Install targets
install(TARGETS
  SensorBridge
  arm_control_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
