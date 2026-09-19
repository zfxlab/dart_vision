cmake_minimum_required(VERSION 3.8)
project(dart_stereo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(stereo_core src/bearing_solver.cpp src/stereo_triangulator.cpp)
target_include_directories(stereo_core PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stereo_core ${OpenCV_LIBS})
add_executable(stereo_node src/stereo_triangulator_main.cpp src/stereo_triangulator_node.cpp)
target_link_libraries(stereo_node stereo_core)
ament_target_dependencies(stereo_node rclcpp sensor_msgs dart_interfaces geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS stereo_core stereo_node ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
