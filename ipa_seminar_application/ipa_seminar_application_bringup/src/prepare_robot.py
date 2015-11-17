#!/usr/bin/python
import rospy

from simple_script_server import *
sss = simple_script_server()
		
if __name__ == "__main__":
	rospy.init_node("prepare_robot")
	rospy.sleep(3)

	# move robot components concurrent
	handle_arm = sss.move("arm","home")
