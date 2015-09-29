#!/usr/bin/env python
import rospy
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionResult

class fake_gripper():

    def __init__(self):
        self._as = actionlib.SimpleActionServer("/gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self.pub = rospy.Publisher('/joint_states',JointState)
        self.position = [0.0, 0.0]

    def execute_cb(self,goal):
        rospy.sleep(5)
        print goal
        self.position = goal
        self._as.set_succeeded(FollowJointTrajectoryActionResult)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["gripper_finger_left_joint", "gripper_finger_right_joint"]
        msg.position = self.position
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]
        self.pub.publish(msg)

if __name__ == '__main__':
        try:
                rospy.init_node('fake_gripper')
                fg = fake_gripper()
                while not rospy.is_shutdown():
                    fg.publish_joint_states()
                    rospy.sleep(0.1)
        except rospy.ROSInterruptException:
                print "Interupted"
                pass
