cmake_minimum_required(VERSION 3.8)
project(fanuc_driver)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WeldCommand.msg"
  "msg/WeldState.msg"
  "msg/RobotStatus.msg"
  "msg/JointPosition.msg"
  "msg/JointTrajectoryPoint.msg"
  DEPENDENCIES std_msgs sensor_msgs trajectory_msgs
)

# Create fanuc_driver library with common components
add_library(fanuc_driver_lib SHARED
  src/simple_message.cpp
  src/fanuc_utils.cpp
)
target_include_directories(fanuc_driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fanuc_driver_lib
  rclcpp
  trajectory_msgs
)
rosidl_target_interfaces(fanuc_driver_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Robot state node
add_executable(robot_state_node src/fanuc_robot_state_node.cpp)
target_link_libraries(robot_state_node fanuc_driver_lib)
ament_target_dependencies(robot_state_node
  rclcpp
  sensor_msgs
)
rosidl_target_interfaces(robot_state_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Joint streamer node
add_executable(joint_streamer_node src/fanuc_joint_streamer_node.cpp)
target_link_libraries(joint_streamer_node fanuc_driver_lib)
ament_target_dependencies(joint_streamer_node
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  control_msgs
)
rosidl_target_interfaces(joint_streamer_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(weld_command_node src/fanuc_weld_command_node.cpp)
ament_target_dependencies(weld_command_node rclcpp)
target_link_libraries(weld_command_node yaml-cpp)
rosidl_target_interfaces(weld_command_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS weld_command_node DESTINATION lib/${PROJECT_NAME})

add_executable(weld_state_node_tcp src/fanuc_weld_state_node.cpp)
ament_target_dependencies(weld_state_node_tcp rclcpp)
target_link_libraries(weld_state_node_tcp yaml-cpp)
rosidl_target_interfaces(weld_state_node_tcp ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS weld_state_node_tcp DESTINATION lib/${PROJECT_NAME})

add_executable(weld_state_node_simple src/fanuc_weld_state_node_simple.cpp)
ament_target_dependencies(weld_state_node_simple rclcpp)
target_link_libraries(weld_state_node_simple yaml-cpp)
rosidl_target_interfaces(weld_state_node_simple ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS weld_state_node_simple DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_export_libraries(fanuc_driver_lib)
ament_export_include_directories(include)
ament_package()

# Install library
install(TARGETS fanuc_driver_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS 
  robot_state_node
  joint_streamer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/weld_command_test.py
  scripts/weld_control_freq_test.py
  scripts/test_robot_motion.py
  scripts/simple_motion_test.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Python-based launch test verifying TCP packet emitted by node
  ament_add_pytest_test(test_weld_command_node
    test/test_weld_command_node.py
    TIMEOUT 30
  )
  # Python-based launch test for weld_state_node_tcp parsing
  ament_add_pytest_test(test_weld_state_node
    test/test_weld_state_node.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_weld_state_node_simple
    test/test_weld_state_node_simple.py
    TIMEOUT 30
  )
  # Robot control/state tests
  ament_add_pytest_test(test_robot_state_node
    test/test_robot_state_node.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_simple_message
    test/test_simple_message.py
    TIMEOUT 30
  )
endif()
