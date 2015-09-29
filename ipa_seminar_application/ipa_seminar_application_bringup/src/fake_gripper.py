#!/usr/bin/env python
import rospy
from brics_showcase_industry_interfaces.srv import MoveGripper, MoveGripperResponse	

def handle_move_gripper(req):
    print "moving gripper to " + str(req.open)
    return MoveGripperResponse()

def move_gripper_server():
    rospy.init_node('fake_gripper')
    s = rospy.Service('/MoveGripper', MoveGripper, handle_move_gripper)
    print "Ready to move the gripper."
    rospy.spin()

if __name__ == "__main__":
    move_gripper_server()
