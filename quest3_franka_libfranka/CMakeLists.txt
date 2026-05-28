cmake_minimum_required(VERSION 3.8)
project(quest3_franka_libfranka)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(libfranka_cartesian_pose_node
  src/libfranka_cartesian_pose_node.cpp
)

target_include_directories(libfranka_cartesian_pose_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
)

ament_target_dependencies(libfranka_cartesian_pose_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(libfranka_cartesian_pose_node
  ${Franka_LIBRARIES}
)

install(
  TARGETS libfranka_cartesian_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
