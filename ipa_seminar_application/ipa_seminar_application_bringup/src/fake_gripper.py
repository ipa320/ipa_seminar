#!/usr/bin/env python
import rospy
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult, JointTrajectoryControllerState

class fake_gripper():

    def __init__(self):
        self._as = actionlib.SimpleActionServer("/gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self.pub = rospy.Publisher('/gripper_controller/joint_states', JointState, queue_size=1)
        self.pub_state = rospy.Publisher('/gripper/joint_trajectory_controller/state', JointTrajectoryControllerState, queue_size=1)
        self.position = [0.0, 0.0]

    def execute_cb(self, goal):
        #rospy.sleep(goal.trajectory.points[-1].time_from_start)
        rospy.sleep(1.0)
        print goal
        self.position = goal.trajectory.points[-1].positions
        self._as.set_succeeded(FollowJointTrajectoryResult(error_code = 0))

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["gripper_finger_left_joint", "gripper_finger_right_joint"]
        msg.position = self.position
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]
        msg_state = JointTrajectoryControllerState()
        msg_state.actual.positions = self.position
        self.pub.publish(msg)
        self.pub_state.publish(msg_state)

if __name__ == '__main__':
        rospy.init_node('fake_gripper')
        fg = fake_gripper()
        try:
                while not rospy.is_shutdown():
                    fg.publish_joint_states()
                    rospy.sleep(0.1)
        except rospy.ROSInterruptException:
                print "Interupted"
                pass
