cmake_minimum_required(VERSION 2.8.3)
project(macaron)

find_package(catkin REQUIRED COMPONENTS
#for communication
  message_generation
  roscpp
  serial
  std_msgs

#for path planning and localization
  nav_msgs
  geometry_msgs

#for recognigation
  sensor_msgs
  cv_bridge
  image_transport
  rospy

)
find_package( OpenCV REQUIRED )

catkin_python_setup()

 add_message_files(
	FILES
	Floats.msg
	Floats_for_mission.msg
	HeaderString.msg
	erp42_read.msg
	erp42_write.msg
	spoon.msg
	Message2.msg
	base_frame.msg
 )

 generate_messages(
   DEPENDENCIES
   sensor_msgs
   nav_msgs
   geometry_msgs
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES macaron
  CATKIN_DEPENDS message_generation rospy serial std_msgs roscpp sensor_msgs message_runtime
)

include_directories(
  ${OPENCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



#add_executable(path_planning src/path_planning.cpp)
#add_dependencies(path_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(path_planning ${catkin_LIBRARIES})

add_executable(serial_example_node src/serial_example_node.cpp)
add_dependencies(serial_example_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(serial_example_node ${catkin_LIBRARIES})

add_executable(vel_planning src/vel_planning.cpp)
add_dependencies(vel_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vel_planning ${catkin_LIBRARIES})

add_executable(traffic_test src/traffic_test.cpp)
add_dependencies(traffic_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(traffic_test ${catkin_LIBRARIES})

#add_executable(puppet src/puppet.cpp)
#add_dependencies(puppet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(puppet ${catkin_LIBRARIES})

add_executable(path_planning src/path_planning.cpp)
add_dependencies(path_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planning ${catkin_LIBRARIES})

add_executable(path_planning_mul src/path_planning_mul.cpp)
add_dependencies(path_planning_mul ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planning_mul ${catkin_LIBRARIES})

add_executable(remote_control src/remote_control.cpp)
add_dependencies(remote_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(remote_control ${catkin_LIBRARIES})

add_executable(mission_identifying src/mission_identifying.cpp)
add_dependencies(mission_identifying ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mission_identifying ${catkin_LIBRARIES})

add_executable(parking_algorithm src/parking_algorithm.cpp)
add_dependencies(parking_algorithm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(parking_algorithm ${catkin_LIBRARIES})

add_executable(serial_output src/serial_output.cpp)
add_dependencies(serial_output ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(serial_output ${catkin_LIBRARIES})


#target_link_libraries(linetracker #
#	${OPENCV_LIBRARIES}	##
#	${catkin_LIBRARIES})

catkin_install_python(PROGRAMS 
  src/lanetracker.py
  src/lanetracker2.py
  src/path_planner.py
  src/GPSTxT_reader.py
  src/mp42bag.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



