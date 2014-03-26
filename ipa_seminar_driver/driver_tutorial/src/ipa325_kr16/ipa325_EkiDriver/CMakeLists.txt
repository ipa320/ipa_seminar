cmake_minimum_required(VERSION 2.8.3)
project(ipa325_eki_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	actionlib
	actionlib_msgs
	geometry_msgs
	ipa325_msgs
	roscpp
	rospy
	sensor_msgs
	control_msgs
	tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

find_library(POCO_NET PocoNet)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS common/include
  LIBRARIES ${POCO_NET} KukaEthernetClient_lib pugixml_lib2
  CATKIN_DEPENDS 	
	actionlib
	actionlib_msgs
	geometry_msgs
	ipa325_msgs
	roscpp
	rospy
	sensor_msgs
	control_msgs
	tf
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include common/include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(pugixml_lib2
	common/src/pugixml/pugixml.cpp
)

add_library(KukaEthernetClient_lib
	common/src/KukaEthernetClient/KukaEthernetClient.cpp
	common/src/KukaEthernetClient/KukaAxis.cpp
	common/src/KukaEthernetClient/KukaFrame.cpp
	common/src/KukaEthernetClient/StringEthernetClient.cpp
	common/src/KukaEthernetClient/XmlStringHandler.cpp
)

## Declare a cpp executable
add_executable(EkiDriver 
	src/EkiDriver.cpp
)

add_executable(EkiTestClient 
	src/EkiTestClient.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(EkiDriver ipa325_msgs_gencpp)
add_dependencies(EkiTestClient ipa325_msgs_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(EkiDriver
	${catkin_LIBRARIES}
	${POCO_NET}
	${Boost_LIBRARIES}
	KukaEthernetClient_lib
	pugixml_lib2
)

target_link_libraries(EkiTestClient
	${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS dummy_pkg dummy_pkg_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dummy_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
