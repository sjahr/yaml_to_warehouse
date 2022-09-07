cmake_minimum_required(VERSION 3.22)
project(yaml_to_warehouse LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(moveit_ros_warehouse REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(warehouse_ros REQUIRED)

find_library(FMT fmt)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  yaml-cpp
  Eigen3
  std_msgs
  geometry_msgs
  geometric_shapes
  moveit_ros_warehouse
  moveit_msgs
  warehouse_ros
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(include)

add_executable(yaml_to_warehouse src/yaml_msg_convert.cpp src/geometry.cpp src/yaml_to_warehouse.cpp)
target_link_libraries(yaml_to_warehouse yaml-cpp ${FMT})
ament_target_dependencies(yaml_to_warehouse
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(
  TARGETS
    yaml_to_warehouse
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
