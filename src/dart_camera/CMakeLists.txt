cmake_minimum_required(VERSION 3.8)
project(dart_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(dart_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

add_library(dart_camera_core src/green_light_detector.cpp)
target_include_directories(dart_camera_core PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(dart_camera_core ${OpenCV_LIBS})
add_executable(green_light_detector_node src/green_light_detector_main.cpp src/green_light_detector_node.cpp)
target_link_libraries(green_light_detector_node dart_camera_core)
ament_target_dependencies(green_light_detector_node rclcpp rcl_interfaces sensor_msgs cv_bridge dart_interfaces)

install(TARGETS dart_camera_core green_light_detector_node ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
