cmake_minimum_required(VERSION 3.8)
project(fanuc_driver)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WeldCommand.msg"
  "msg/WeldState.msg"
  DEPENDENCIES std_msgs
)

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
ament_package()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/weld_command_test.py
  scripts/weld_control_freq_test.py
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
endif()
