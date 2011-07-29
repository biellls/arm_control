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

include_directories(${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME})

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_library(${PROJECT_NAME} src/serialcomm.cpp src/melfa.cpp)
rosbuild_add_executable(execute src/tools/execute.cpp)
rosbuild_add_executable(batch_execute src/tools/batch_execute.cpp)
rosbuild_add_executable(send_raw src/tools/send_raw.cpp)
rosbuild_add_executable(batch_send_raw src/tools/batch_send_raw.cpp)
rosbuild_add_executable(arm_control_node src/nodes/arm_control_node.cpp)
rosbuild_add_executable(follow_path src/nodes/follow_path.cpp)
target_link_libraries(execute ${PROJECT_NAME})
target_link_libraries(batch_execute ${PROJECT_NAME})
target_link_libraries(send_raw ${PROJECT_NAME})
target_link_libraries(batch_send_raw ${PROJECT_NAME})
target_link_libraries(arm_control_node ${PROJECT_NAME})

#common commands for building c++ executables and libraries
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
