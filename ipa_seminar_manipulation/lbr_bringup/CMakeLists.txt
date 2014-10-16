cmake_minimum_required(VERSION 2.8.3)
project(lbr_bringup)

find_package(catkin REQUIRED)

catkin_package()


# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
install(PROGRAMS scripts/scripting_template.py scripts/scripting_example.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
