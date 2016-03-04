cmake_minimum_required(VERSION 2.8.3)
project(arm_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  actionlib_msgs
  actionlib
  std_msgs
  geometry_msgs
  sensor_msgs
  tf
  nav_msgs)
find_package(Boost REQUIRED COMPONENTS program_options system)

if (UNIX)
  find_library(YAMLCPP_LIBRARY NAMES yaml-cpp)
  find_file(YAMLCPP_HAVE_H yaml-cpp/yaml.h )
  find_path(YAMLCPP_H_INCLUDE_DIR yaml-cpp/yaml.h )
  SET(YAMLCPP_INCLUDE_DIR ${YAMLCPP_H_INCLUDE_DIR}/yaml-cpp)
else()
  find_library(
    YAMLCPP_LIBRARY
    NAMES yamlcpp.lib
    NO_DEFAULT_PATH
    PATHS
    $ENV{YAMLCPP_ROOT}/lib
  )
  find_file(YAMLCPP_HAVE_H include/yaml.h PATHS $ENV{YAMLCPP_ROOT} NO_DEFAULT_PATH)
  find_path(YAMLCPP_H_INCLUDE_DIR include/yaml.h PATHS $ENV{YAMLCPP_ROOT} NO_DEFAULT_PATH)
  SET(YAMLCPP_INCLUDE_DIR ${YAMLCPP_H_INCLUDE_DIR}/include)
endif(UNIX)

SET(YAMLCPP_FOUND FALSE)
if (YAMLCPP_HAVE_H)
  if (YAMLCPP_LIBRARY)
    SET(YAMLCPP_FOUND TRUE)
  endif (YAMLCPP_LIBRARY)
endif (YAMLCPP_HAVE_H)

if(NOT YAMLCPP_FOUND)
  message(STATUS "YamlCpp Can't be found")
  break()
else()
  message(STATUS "YamlCpp Library: " ${YAMLCPP_LIBRARY} )
  message(STATUS "YamlCpp Header: " ${YAMLCPP_INCLUDE_DIR} )
endif(NOT YAMLCPP_FOUND)

add_service_files(FILES
  SetTool.srv
)

add_action_files(DIRECTORY action FILES MoveJoints.action MoveTool.action)

generate_messages(DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
  sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES melfa melfa_ros serial
  CATKIN_DEPENDS roscpp message_runtime actionlib_msgs actionlib std_msgs geometry_msgs sensor_msgs tf nav_msgs
  DEPENDS Boost)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})

add_library(melfa src/melfa/melfa.cpp
                  src/melfa/joint_state.cpp
                  src/melfa/tool_pose.cpp)
add_library(serial src/serial/serialcomm.cpp)
add_library(melfa_ros src/melfa_ros/robot_path.cpp
                      src/melfa_ros/conversions.cpp)
add_executable(execute src/tools/execute.cpp)
add_executable(batch_execute src/tools/batch_execute.cpp)
add_executable(send_raw src/tools/send_raw.cpp)
add_executable(batch_send_raw src/tools/batch_send_raw.cpp)
add_executable(move src/tools/move.cpp)
add_executable(create_pose_msg src/tools/create_pose_msg.cpp)
add_executable(arm_control_node src/nodes/arm_control_node.cpp)
add_executable(follow_path_node src/nodes/follow_path_node.cpp)
add_executable(execute_node src/nodes/execute_node.cpp)
target_link_libraries(melfa serial ${catkin_LIBRARIES})
target_link_libraries(execute melfa)
target_link_libraries(batch_execute melfa)
target_link_libraries(send_raw melfa)
target_link_libraries(batch_send_raw melfa)
target_link_libraries(move melfa ${Boost_LIBRARIES})
target_link_libraries(move ${Boost_LIBRARIES})
target_link_libraries(melfa_ros yaml-cpp ${catkin_LIBRARIES})
target_link_libraries(arm_control_node melfa
                                       melfa_ros)
add_dependencies(arm_control_node arm_control_gen_cpp)
target_link_libraries(follow_path_node melfa_ros)
target_link_libraries(execute_node melfa ${catkin_LIBRARIES})
add_dependencies(execute_node execute_gen_cpp)
target_link_libraries(create_pose_msg melfa_ros)

catkin_add_gtest(melfa_ros_test test/melfa_ros_test.cpp)
target_link_libraries(melfa_ros_test melfa_ros)
