cmake_minimum_required(VERSION 3.8)
project(dart_aiming)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(aiming_core src/aiming_core.cpp)
target_include_directories(aiming_core PUBLIC include)
add_executable(aiming_node src/aiming_main.cpp src/aiming_node.cpp)
target_link_libraries(aiming_node aiming_core)
ament_target_dependencies(aiming_node rclcpp rcl_interfaces dart_interfaces geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS aiming_core aiming_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
