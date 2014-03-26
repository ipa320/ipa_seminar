cmake_minimum_required(VERSION 2.8.3)
project(kr16_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
  	rospy
  	std_msgs
    actionlib
    dynamic_reconfigure 
	sensor_msgs
	control_msgs
	ipa325_eki_driver
)

# protected region additional user defined REQUIREMENTS on begin #
# protected region additional user defined REQUIREMENTS end #

generate_dynamic_reconfigure_options(cfg/kr16_node.cfg)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    common/src
  CATKIN_DEPENDS 
  	roscpp
  	rospy
  	std_msgs
    dynamic_reconfigure
	sensor_msgs
	control_msgs
	ipa325_eki_driver
   
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(common/include)
include_directories(common/src)
include_directories(${catkin_INCLUDE_DIRS})


## Declare a cpp executable
add_executable(kr16_node ros/src/kr16_node_ros.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(kr16_node kr16_node_gencfg)
add_dependencies(kr16_node ${kr16_driver_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(kr16_node
  ${catkin_LIBRARIES}
)

# protected region additional user defined BUILD STATEMENTS on begin #
#target_link_libraries(kr16_node
#  KukaEthernetClient_lib
#)
# protected region additional user defined BUILD STATEMENTS end #


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS kr16_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# protected region user Cmake macros on begin #
# protected region user Cmake macros end #
