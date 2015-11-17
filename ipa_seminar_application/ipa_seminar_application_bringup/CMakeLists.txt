# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(ipa_seminar_application_bringup)

find_package(catkin REQUIRED COMPONENTS roslaunch rostest)

# rostest for robots
set(robotlist
ur
lwa4d
)

# loop through list of robots for testing
foreach(robot ${robotlist})
#  message("testing for robot: ${robot}")
#  roslaunch_add_file_check(launch robot:=${robot})
#  add_rostest(test/urdf_${robot}.test)
endforeach(robot)

catkin_package()

