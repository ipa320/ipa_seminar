#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

def fake_publisher():
        rospy.init_node('fake_joint_state_publisher_connector')

        pub = rospy.Publisher('/joint_states',JointState)
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ['gripper_finger_joint','gripper_finger_mimic_joint']
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
