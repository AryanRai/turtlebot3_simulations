################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(turtlebot3_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

################################################################################
# Build
################################################################################
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${GZ_SIM_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${gz_sim_vendor_INCLUDE_DIRS}
  ${GZ_SIM_INCLUDE_DIRS}
)

set(EXEC_NAME "turtlebot3_drive")

add_executable(${EXEC_NAME} 
  src/turtlebot3_drive.cpp
  src/navigation_controller.cpp
  src/sensor_processor.cpp
  src/motion_controller.cpp
  src/state_machine.cpp
)
target_link_libraries(${EXEC_NAME}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  tf2::tf2
)

# Path visualizer executable
add_executable(path_visualizer
  src/path_visualizer.cpp
)
target_link_libraries(path_visualizer
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_srvs_TARGETS}
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

set(OBSTACLE_LIBS
  obstacles
  obstacle1
  obstacle2
)

foreach(lib ${OBSTACLE_LIBS})
  add_library(${lib} SHARED src/${lib}.cpp)
  target_link_libraries(${lib}
    PUBLIC
    gz-sim::gz-sim
    gz-math::gz-math
  )
endforeach()

################################################################################
# Install
################################################################################
install(TARGETS
  ${EXEC_NAME}
  path_visualizer
  obstacles
  obstacle1
  obstacle2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models params rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(
  gazebo_ros_pkgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
ament_package()
