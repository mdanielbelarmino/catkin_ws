cmake_minimum_required(VERSION 2.8.3)

project(robotic_rover_v2)

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  rospy
  sensor_msgs
  tf
)

catkin_package()

find_package(roslaunch)

include_directories(${catkin_INCLUDE_DIRS})

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

add_executable(rover_odom src/rover_odom.cpp)
target_link_libraries(rover_odom ${catkin_LIBRARIES})