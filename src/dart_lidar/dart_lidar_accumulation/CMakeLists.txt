cmake_minimum_required(VERSION 3.8)
project(dart_lidar_accumulation)

if(CMAKE_COMPILER_IS_GNUCXX
   OR CMAKE_CXX_COMPILER_ID
      MATCHES
      "Clang"
)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(
  Eigen3
  REQUIRED
  NO_MODULE
)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(
  lidar_accumulator_node
  src/lidar_accumulator_main.cpp
  src/lidar_accumulator_config.cpp
  src/lidar_accumulator_node.cpp
)
target_include_directories(
  lidar_accumulator_node
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS}
)
target_link_libraries(
  lidar_accumulator_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)
target_compile_definitions(
  lidar_accumulator_node
  PRIVATE ${PCL_DEFINITIONS}
)
ament_target_dependencies(
  lidar_accumulator_node
  builtin_interfaces
  pcl_conversions
  rclcpp
  rcl_interfaces
  sensor_msgs
)

install(TARGETS lidar_accumulator_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
