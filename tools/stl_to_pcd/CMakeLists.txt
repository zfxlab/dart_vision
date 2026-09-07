cmake_minimum_required(VERSION 3.8)
project(stl_to_pcd)

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
find_package(
  Eigen3
  REQUIRED
  NO_MODULE
)
find_package(
  PCL REQUIRED
  COMPONENTS common
             filters
             io
)

add_library(
  stl_to_pcd_core SHARED
  src/converter.cpp
)
target_include_directories(
  stl_to_pcd_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)
target_include_directories(
  stl_to_pcd_core SYSTEM
  PUBLIC ${PCL_INCLUDE_DIRS}
)
target_link_libraries(
  stl_to_pcd_core
  PUBLIC Eigen3::Eigen ${PCL_LIBRARIES}
)
target_compile_definitions(
  stl_to_pcd_core
  PUBLIC ${PCL_DEFINITIONS}
)

add_executable(
  stl_to_pcd
  src/main.cpp
)
target_link_libraries(
  stl_to_pcd
  PRIVATE stl_to_pcd_core
)

install(DIRECTORY include/ DESTINATION include)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
install(
  TARGETS stl_to_pcd_core stl_to_pcd
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL)
ament_package()
