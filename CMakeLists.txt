cmake_minimum_required(VERSION 2.8.3)

project(xfly)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  mav_msgs
  message_generation
  adrc_control
  pid_control
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  xfly_pose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  # LIBRARIES pid 
  CATKIN_DEPENDS roscpp  sensor_msgs  message_runtime geometry_msgs mav_msgs
  DEPENDS Eigen3
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

add_executable(xfly_pid_controller_node src/xfly_pid_controller_node.cpp)
target_link_libraries(xfly_pid_controller_node ${catkin_LIBRARIES})
add_dependencies(xfly_pid_controller_node ${PROJECT_NAME}_generate_messages_cpp)

# add_executable(omni_adrc_controller_node src/omni_adrc_controller_node.cpp)
# target_link_libraries(omni_adrc_controller_node  adrc ${catkin_LIBRARIES})
# add_dependencies(omni_adrc_controller_node ${vfly}_generate_messages_cpp)

# add_executable(rotation_on_fixed_spot_example src/rotation_on_fixed_spot_example.cpp)
# target_link_libraries(rotation_on_fixed_spot_example ${catkin_LIBRARIES})
# add_dependencies(rotation_on_fixed_spot_example ${vfly}_generate_messages_cpp)

# add_executable(move_with_zero_attitude_example src/move_with_zero_attitude_example.cpp)
# target_link_libraries(move_with_zero_attitude_example ${catkin_LIBRARIES})
# add_dependencies(move_with_zero_attitude_example ${vfly}_generate_messages_cpp)