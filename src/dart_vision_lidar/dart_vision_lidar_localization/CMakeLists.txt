cmake_minimum_required(VERSION 3.8)
project(dart_vision_lidar_localization)

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
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(dart_vision_interfaces REQUIRED)
find_package(
  Eigen3
  REQUIRED
  NO_MODULE
)
find_package(geometry_msgs REQUIRED)
find_package(livox_interfaces REQUIRED)
find_package(
  PCL
  QUIET
  REQUIRED
  COMPONENTS common
             filters
             io
             kdtree
             registration
             search
)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(
  ${PROJECT_NAME}_core SHARED
  src/core/alignment_metrics.cpp
  src/core/cloud_utils.cpp
  src/core/temporal_gate.cpp
  src/localizers/armor_localizer.cpp
  src/localizers/base_localizer.cpp
  src/pipeline/point_cloud_accumulator.cpp
  src/pipeline/preprocessor.cpp
)
target_include_directories(
  ${PROJECT_NAME}_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)
target_include_directories(
  ${PROJECT_NAME}_core SYSTEM
  PUBLIC ${PCL_INCLUDE_DIRS}
)
target_link_libraries(
  ${PROJECT_NAME}_core
  PUBLIC Eigen3::Eigen ${PCL_LIBRARIES} Threads::Threads
)
target_compile_definitions(
  ${PROJECT_NAME}_core
  PUBLIC ${PCL_DEFINITIONS}
)

add_library(
  ${PROJECT_NAME}_adapters SHARED
  src/ros/input_adapters.cpp
)
target_include_directories(
  ${PROJECT_NAME}_adapters
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  ${PROJECT_NAME}_adapters ${PROJECT_NAME}_core
)
ament_target_dependencies(
  ${PROJECT_NAME}_adapters
  livox_interfaces
  sensor_msgs
)

add_executable(
  mid70_localization_node
  src/ros/mid70_localization_main.cpp src/ros/mid70_localization_node.cpp
)
target_include_directories(
  mid70_localization_node
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(
  mid70_localization_node
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_adapters
)
ament_target_dependencies(
  mid70_localization_node
  dart_vision_interfaces
  geometry_msgs
  livox_interfaces
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
  visualization_msgs
)

install(DIRECTORY include/ DESTINATION include)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}_core
          ${PROJECT_NAME}_adapters
          mid70_localization_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  dart_vision_interfaces
  Eigen3
  geometry_msgs
  livox_interfaces
  PCL
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
  visualization_msgs
)

ament_package()
