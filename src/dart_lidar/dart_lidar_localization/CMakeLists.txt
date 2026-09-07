cmake_minimum_required(VERSION 3.8)
project(dart_lidar_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(
  dart_lidar_localization_core SHARED
  src/localization_types.cpp
  src/base/base_registrar.cpp
  src/base/base_validator.cpp
  src/base/cloud_pyramid.cpp
  src/base/gicp_registrar.cpp
  src/base/ndt_registrar.cpp
  src/io/model_loader.cpp
  src/io/result_writer.cpp
  src/module/module_validator.cpp
  src/module/rail_module_localizer.cpp
  src/module/temporal_tracker.cpp
  src/pipeline/localization_pipeline.cpp
)
target_include_directories(
  dart_lidar_localization_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(dart_lidar_localization_core Eigen3::Eigen yaml-cpp ${PCL_LIBRARIES})
target_compile_definitions(dart_lidar_localization_core PRIVATE ${PCL_DEFINITIONS})
ament_target_dependencies(dart_lidar_localization_core ament_index_cpp)

add_executable(
  lidar_localization_node
  src/ros/lidar_localization_main.cpp
  src/ros/lidar_localization_node.cpp
)
target_include_directories(
  lidar_localization_node
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS}
)
target_link_libraries(lidar_localization_node dart_lidar_localization_core)
target_compile_definitions(lidar_localization_node PRIVATE ${PCL_DEFINITIONS})
ament_target_dependencies(
  lidar_localization_node
  dart_interfaces
  geometry_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
)

install(
  TARGETS dart_lidar_localization_core lidar_localization_node
  EXPORT dart_lidar_localization_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(dart_lidar_localization_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_index_cpp Eigen3 PCL yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_base_validator test/test_base_validator.cpp)
  target_link_libraries(test_base_validator dart_lidar_localization_core)

  ament_add_gtest(test_module_localization test/test_module_localization.cpp)
  target_link_libraries(test_module_localization dart_lidar_localization_core)
endif()

ament_package()
