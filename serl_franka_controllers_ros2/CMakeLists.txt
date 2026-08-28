cmake_minimum_required(VERSION 3.8)
project(serl_franka_controllers_ros2)

if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

set(CMAKE_AUTOMOC ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(Franka REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ZeroJacobian.msg"
  "msg/CartesianImpedanceCommand.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

set(controller_library ${PROJECT_NAME}_controllers)
set(impedance_core_library ${PROJECT_NAME}_impedance_core)

add_library(${impedance_core_library} STATIC
  src/cartesian_impedance_core.cpp
)
set_target_properties(${impedance_core_library} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${impedance_core_library} PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${impedance_core_library} Eigen3::Eigen)

add_library(${controller_library} SHARED
  src/cartesian_impedance_controller.cpp
  src/cartesian_pose_command_controller.cpp
  src/joint_position_controller.cpp
)

add_executable(libfranka_http_tool
  src/libfranka_http_tool.cpp
)

add_library(${PROJECT_NAME}_rviz_panel SHARED
  src/impedance_tuning_panel.cpp
)

target_include_directories(${controller_library} PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
)

ament_target_dependencies(${controller_library}
  controller_interface
  franka_semantic_components
  geometry_msgs
  hardware_interface
  pluginlib
  rcl_interfaces
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
)

target_include_directories(${PROJECT_NAME}_rviz_panel PUBLIC
  include
)

target_include_directories(libfranka_http_tool PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
)

ament_target_dependencies(libfranka_http_tool realtime_tools)

ament_target_dependencies(${PROJECT_NAME}_rviz_panel
  ament_index_cpp
  pluginlib
  rclcpp
  rviz_common
)

target_link_libraries(${PROJECT_NAME}_rviz_panel
  Qt5::Widgets
)

target_link_libraries(libfranka_http_tool
  ${impedance_core_library}
  ${Franka_LIBRARIES}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${controller_library}
  ${impedance_core_library}
  ${Franka_LIBRARIES}
  ${cpp_typesupport_target}
)

pluginlib_export_plugin_description_file(controller_interface serl_franka_controllers_plugin.xml)
pluginlib_export_plugin_description_file(rviz_common serl_franka_controllers_rviz_plugin.xml)

install(
  TARGETS ${controller_library} ${PROJECT_NAME}_rviz_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS libfranka_http_tool
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS
    scripts/serl_franka_http_server.py
    scripts/libfranka_http_server.py
    scripts/test_dual_http_motion.py
    scripts/franka_error_recovery_watchdog.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES serl_franka_controllers_plugin.xml serl_franka_controllers_rviz_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(cartesian_impedance_core_test test/cartesian_impedance_core_test.cpp)
  target_link_libraries(cartesian_impedance_core_test ${impedance_core_library})
  add_test(NAME cartesian_impedance_core_test COMMAND cartesian_impedance_core_test)
  add_test(
    NAME libfranka_dual_http_server_test
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/libfranka_dual_http_server_test.py
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/libfranka_http_server.py
  )
  add_test(
    NAME dual_http_motion_self_test
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_dual_http_motion.py --self-test
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${controller_library})
ament_export_dependencies(
  controller_interface
  ament_index_cpp
  franka_semantic_components
  geometry_msgs
  hardware_interface
  pluginlib
  rcl_interfaces
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  rosidl_default_runtime
  rviz_common
  sensor_msgs
  std_msgs
)

ament_package()
