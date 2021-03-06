cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

rosbuild_add_boost_directories()

rosbuild_add_library(melfa src/melfa/melfa.cpp src/melfa/joint_state.cpp src/melfa/tool_pose.cpp)
rosbuild_add_library(serial src/serial/serialcomm.cpp)
rosbuild_add_library(melfa_ros src/melfa_ros/robot_path.cpp src/melfa_ros/conversions.cpp)
rosbuild_add_executable(execute src/tools/execute.cpp)
rosbuild_add_executable(batch_execute src/tools/batch_execute.cpp)
rosbuild_add_executable(send_raw src/tools/send_raw.cpp)
rosbuild_add_executable(batch_send_raw src/tools/batch_send_raw.cpp)
rosbuild_add_executable(move src/tools/move.cpp)
rosbuild_add_executable(create_pose_msg src/tools/create_pose_msg.cpp)
rosbuild_add_executable(arm_control_node src/nodes/arm_control_node.cpp)
rosbuild_add_executable(follow_path_node src/nodes/follow_path_node.cpp)
target_link_libraries(melfa serial)
target_link_libraries(execute melfa)
target_link_libraries(batch_execute melfa)
target_link_libraries(send_raw melfa)
target_link_libraries(batch_send_raw melfa)
target_link_libraries(move melfa)
rosbuild_link_boost(move program_options)
target_link_libraries(melfa_ros yaml-cpp)
target_link_libraries(arm_control_node melfa melfa_ros)
target_link_libraries(follow_path_node melfa_ros yaml-cpp)
target_link_libraries(create_pose_msg melfa_ros)

rosbuild_add_gtest(melfa_ros_test test/melfa_ros_test.cpp)
target_link_libraries(melfa_ros_test melfa_ros)

#common commands for building c++ executables and libraries
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

