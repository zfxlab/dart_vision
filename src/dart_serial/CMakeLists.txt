cmake_minimum_required(VERSION 3.8)
project(dart_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(
  serial_core
  src/crc.cpp
  src/packet.cpp
  src/packet_parser.cpp
  src/serial_port.cpp
)

target_include_directories(
  serial_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>
)

add_executable(
  serial_node
  src/serial_main.cpp
  src/serial_node.cpp
)

target_include_directories(
  serial_node
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(serial_node serial_core)
ament_target_dependencies(serial_node rclcpp dart_interfaces sensor_msgs)

install(
  TARGETS serial_core serial_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
