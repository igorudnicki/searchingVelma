cmake_minimum_required(VERSION 3.5)
project(searcher)
set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED COMPONENTS
	rospy
	roscpp
	std_msgs
	geometry_msgs
	message_generation
	tf
	costmap_2d
	global_planner
	kdl_conversions
	base_local_planner
	dwa_local_planner
	sensor_msgs
	cv_bridge
)
include_directories(${catkin_INCLUDE_DIRS})
add_service_files(
  FILES
  Lookup.srv
  Stpt.srv
  Beer.srv
  Opening.srv
  Detect.srv
  Segment.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
 CATKIN_DEPENDS message_runtime
)

install (DIRECTORY worlds/ DESTINATION share/${CMAKE_PROJECT_NAME}/worlds 
         FILES_MATCHING PATTERN "*.world")
install (DIRECTORY launch/ DESTINATION share/${CMAKE_PROJECT_NAME}/launch 
         FILES_MATCHING PATTERN "*.launch*")
install (DIRECTORY config/ DESTINATION share/${CMAKE_PROJECT_NAME}/config 
         FILES_MATCHING PATTERN "*.yaml")
install (DIRECTORY param/ DESTINATION share/${CMAKE_PROJECT_NAME}/param 
         FILES_MATCHING PATTERN "*.yaml")
install (DIRECTORY param/ DESTINATION share/${CMAKE_PROJECT_NAME}/srv 
         FILES_MATCHING PATTERN "*.srv")

add_executable(move_base cppscripts/move_base.cpp)
target_link_libraries(move_base ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/beer_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/lookup_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/opening_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/detection_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(cylinder_segmentation)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cylinder_segmentation cppscripts/cylinder_segmentation.cpp)

target_link_libraries(cylinder_segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES})

