cmake_minimum_required(VERSION 3.0.2)
project(fanuc_driver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# let industrial_robot_client pull in all other dependencies
find_package(catkin REQUIRED COMPONENTS 
  industrial_robot_client
  message_generation
  std_msgs)


# 为不同的构建类型设置编译标志
# Debug 模式: 调试信息和所有警告
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-unused-parameter")

# Release 模式: 优化和基础警告
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG -Wall -Wextra")

# Add message files
add_message_files(
  FILES
  WeldState.msg
  WeldCommand.msg
)

# Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    industrial_robot_client
    message_runtime
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# non-swapping Fanuc specific robot state node
add_executable(${PROJECT_NAME}_robot_state
  src/fanuc_robot_state_node.cpp
  src/fanuc_utils.cpp)
target_link_libraries(${PROJECT_NAME}_robot_state
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_robot_state
  PROPERTIES OUTPUT_NAME robot_state
  PREFIX "")
target_compile_definitions(
  ${PROJECT_NAME}_robot_state
  PRIVATE
  ${industrial_robot_client_DEFINITIONS})


# byte-swapping Fanuc specific robot state node
add_executable(${PROJECT_NAME}_robot_state_bswap
  src/fanuc_robot_state_node.cpp
  src/fanuc_utils.cpp)
target_link_libraries(${PROJECT_NAME}_robot_state_bswap
  industrial_robot_client_bswap
  simple_message_bswap
  ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_robot_state_bswap
  PROPERTIES OUTPUT_NAME robot_state_bswap
  PREFIX "")
target_compile_definitions(
  ${PROJECT_NAME}_robot_state_bswap
  PRIVATE
  ${industrial_robot_client_bswap_DEFINITIONS})


# TCP version Weld state node (direct TCP connection)
add_executable(${PROJECT_NAME}_weld_state_tcp
  src/fanuc_weld_state_node.cpp)
target_link_libraries(${PROJECT_NAME}_weld_state_tcp
  ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_weld_state_tcp
  PROPERTIES OUTPUT_NAME fanuc_weld_state_node_tcp
  PREFIX "")
add_dependencies(${PROJECT_NAME}_weld_state_tcp ${PROJECT_NAME}_generate_messages_cpp)

# Simple Message version Weld state node (ROS-Industrial protocol)
add_executable(${PROJECT_NAME}_weld_state_simple
  src/fanuc_weld_state_node_simple.cpp)
target_link_libraries(${PROJECT_NAME}_weld_state_simple
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_weld_state_simple
  PROPERTIES OUTPUT_NAME fanuc_weld_state_node_simple
  PREFIX "")
target_compile_definitions(
  ${PROJECT_NAME}_weld_state_simple
  PRIVATE
  ${industrial_robot_client_DEFINITIONS})
add_dependencies(${PROJECT_NAME}_weld_state_simple ${PROJECT_NAME}_generate_messages_cpp)


# Weld command node
add_executable(${PROJECT_NAME}_weld_command
  src/fanuc_weld_command_node.cpp)
target_link_libraries(${PROJECT_NAME}_weld_command
  ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_weld_command
  PROPERTIES OUTPUT_NAME weld_command_node
  PREFIX "")
add_dependencies(${PROJECT_NAME}_weld_command ${PROJECT_NAME}_generate_messages_cpp)


# non-swapping Fanuc specific motion streaming interface
add_executable(${PROJECT_NAME}_motion_streaming_interface
  src/fanuc_joint_streamer_node.cpp
  src/fanuc_utils.cpp)
target_link_libraries(${PROJECT_NAME}_motion_streaming_interface
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_motion_streaming_interface
  PROPERTIES OUTPUT_NAME motion_streaming_interface
  PREFIX "")
target_compile_definitions(
  ${PROJECT_NAME}_motion_streaming_interface
  PRIVATE
  ${industrial_robot_client_DEFINITIONS})


# byte-swapping Fanuc specific motion streaming interface
add_executable(${PROJECT_NAME}_motion_streaming_interface_bswap
  src/fanuc_joint_streamer_node.cpp
  src/fanuc_utils.cpp)
target_link_libraries(${PROJECT_NAME}_motion_streaming_interface_bswap
  industrial_robot_client_bswap
  simple_message_bswap
  ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_motion_streaming_interface_bswap
  PROPERTIES OUTPUT_NAME motion_streaming_interface_bswap
  PREFIX "")
target_compile_definitions(
  ${PROJECT_NAME}_motion_streaming_interface_bswap
  PRIVATE
  ${industrial_robot_client_bswap_DEFINITIONS})


if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  set(rl_test_deps
    ${PROJECT_NAME}_robot_state
    ${PROJECT_NAME}_robot_state_bswap
    ${PROJECT_NAME}_motion_streaming_interface
    ${PROJECT_NAME}_motion_streaming_interface_bswap
    ${PROJECT_NAME}_weld_state_tcp
    ${PROJECT_NAME}_weld_state_simple)

  if (TARGET joint_trajectory_action)
    # not nice, but industrial_robot_client doesn't prefix its targets
    list(APPEND rl_test_deps joint_trajectory_action)
  endif()

  roslaunch_add_file_check(tests/roslaunch_test.xml
    DEPENDENCIES ${rl_test_deps})
endif()


install(TARGETS
  ${PROJECT_NAME}_robot_state
  ${PROJECT_NAME}_robot_state_bswap
  ${PROJECT_NAME}_motion_streaming_interface
  ${PROJECT_NAME}_motion_streaming_interface_bswap
  ${PROJECT_NAME}_weld_state_tcp
  ${PROJECT_NAME}_weld_state_simple

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install Python scripts so they can be invoked with rosrun
install(PROGRAMS
  scripts/byte_order_analyzer.py
  scripts/debug_weld_state.py
  scripts/raw_message_capture.py
  scripts/test_debug_tools.py
  scripts/test_weld_state.py
  scripts/weld_command_test.py
  scripts/weld_state_validator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY karel launch tpe
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES readme.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
