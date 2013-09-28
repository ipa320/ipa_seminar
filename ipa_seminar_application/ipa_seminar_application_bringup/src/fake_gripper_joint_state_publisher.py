#!/usr/bin/env python
import roslib; roslib.load_manifest('ipa_seminar_application_bringup')
import rospy
import time
from sensor_msgs.msg import JointState

def fake_publisher():
        rospy.init_node('fake_joint_state_publisher_connector')

        pub = rospy.Publisher('/joint_states',JointState)
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ['gripper_finger_1_joint', 'gripper_finger_2_joint']
        msg.position = [0.0, 0.0]
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]

        while not rospy.is_shutdown():
                pub.publish(msg)
                rospy.sleep(0.1)

if __name__ == '__main__':
        try:
                fake_publisher()
        except rospy.ROSInterruptException:
                print "Interupted"
                pass
