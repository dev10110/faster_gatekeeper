cmake_minimum_required(VERSION 3.10)

project(gatekeeper)

add_definitions(-std=c++11)

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")

set(CMAKE_BUILD_TYPE Release)




find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	message_generation
	pcl_ros
)

catkin_python_setup()

add_service_files(FILES
	MinDist.srv
)

generate_messages(DEPENDENCIES
	geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})


catkin_package(
	INCLUDE_DIRS  include
	CATKIN_DEPENDS 
		roscpp
		rospy
	       	message_runtime
		std_msgs
		geometry_msgs
)

catkin_install_python(PROGRAMS scripts/gatekeeper_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## ADD CPP FILE
add_executable(
	minDist src/mindist.cpp
	)
target_link_libraries(minDist
	${catkin_LIBRARIES}
)
add_dependencies(
	minDist 
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
	gatekeeper_gencpp
)



install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
