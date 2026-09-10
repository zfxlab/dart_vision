cmake_minimum_required(VERSION 3.8)
project(dart_lidar_preprocessing)

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
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(livox_interfaces REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(
  livox_preprocessor_node
  src/livox_preprocessor_main.cpp src/livox_preprocessor_node.cpp
)
target_include_directories(
  livox_preprocessor_node
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS}
)
target_link_libraries(livox_preprocessor_node Eigen3::Eigen ${PCL_LIBRARIES})
target_compile_definitions(
  livox_preprocessor_node
  PRIVATE ${PCL_DEFINITIONS}
)
ament_target_dependencies(
  livox_preprocessor_node
  livox_interfaces
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2
  tf2_eigen
  tf2_ros
)

install(TARGETS livox_preprocessor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
