cmake_minimum_required(VERSION 3.8)
project(dart_stereo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(stereo_core src/stereo_triangulator.cpp)
target_include_directories(stereo_core PUBLIC include)
add_executable(stereo_node src/stereo_triangulator_main.cpp src/stereo_triangulator_node.cpp)
target_link_libraries(stereo_node stereo_core)
ament_target_dependencies(stereo_node rclcpp dart_interfaces sensor_msgs)

install(TARGETS stereo_core stereo_node ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
