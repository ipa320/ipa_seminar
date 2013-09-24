#!/usr/bin/python

import roslib; roslib.load_manifest('ipa_seminar_application')
import rospy

import smach
import smach_ros

from tf.transformations import *
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface


class prepare_robot(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed'])

	def execute(self, userdata):
		print "preparing robot"
		rospy.sleep(3)
		print "robot prepared"
		return 'succeeded'

class pick_object(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['object_picked', 'object_not_picked', 'failed'])
		### Create a handle for the Planning Scene Interface
		self.psi = PlanningSceneInterface()
		### Create a handle for the Move Group Commander
		self.mgc = MoveGroupCommander("arm")

	def execute(self, userdata):
		print "picking object"

		### Add virtual obstacle
		pose = gen_pose(pos=[0.2, 0.1, 1.2])
		self.psi.add_box("box", pose, size=(0.15, 0.15, 0.6))
		rospy.sleep(1.0)
	
		### Move to Cartesian position
		goal_pose = gen_pose(pos=[0.117, -0.600, 1.738], euler=[3.047, 1.568, 3.047])
		self.mgc.go(goal_pose.pose)
	
		### Move Cartesian linear
		goal_pose.pose.position.z -= 0.1
		(traj,frac) = self.mgc.compute_cartesian_path([goal_pose.pose], 0.01, 4, True)
		self.mgc.execute(traj)

		print "object picked"
		return 'object_picked'

class place_object(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['object_placed', 'object_not_placed', 'failed'])
		### Create a handle for the Planning Scene Interface
		self.psi = PlanningSceneInterface()
		### Create a handle for the Move Group Commander
		self.mgc = MoveGroupCommander("arm")

	def execute(self, userdata):
		print "placing object"

		print "object placed"
		return 'object_placed'

class move_ptp(smach.State):
	def __init__(self, position):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed'])
		self.position = position
		### Create a handle for the Planning Scene Interface
		self.psi = PlanningSceneInterface()
		### Create a handle for the Move Group Commander
		self.mgc = MoveGroupCommander("arm")

	def execute(self, userdata):
		print "move ptp to " + self.position

		# get joint_names from parameter server
		param_string = "/pick_and_place/" + self.position
		if not rospy.has_param(param_string):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_string)
			return 'failed'
		param = rospy.get_param(param_string)
		
		# plan trajectory
		goal_pose = gen_pose(pos=param[0], euler=param[1])
		traj = self.mgc.plan(goal_pose.pose)
		if len(traj.joint_trajectory.points) == 0: # TODO is there a better way to know if planning failed or not?
			return 'failed'
		
		# execute trajectory
		self.mgc.execute(traj)

		print "moved ptp to " + str(param)
		return 'succeeded'

class move_lin(smach.State):
	def __init__(self, position):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed'])
		self.position = position
		### Create a handle for the Planning Scene Interface
		self.psi = PlanningSceneInterface()
		### Create a handle for the Move Group Commander
		self.mgc = MoveGroupCommander("arm")

	def execute(self, userdata):
		print "move lin to " + self.position

		# get joint_names from parameter server
		param_string = "/pick_and_place/" + self.position
		if not rospy.has_param(param_string):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_string)
			return 'failed'
		param = rospy.get_param(param_string)
		
		# plan trajectory
		goal_pose = gen_pose(pos=param[0], euler=param[1])
		(traj,frac) = self.mgc.compute_cartesian_path([goal_pose.pose], 0.01, 4, True)
		
		# execute trajectory
		if len(traj.joint_trajectory.points) == 0: # TODO is there a better way to know if planning failed or not?
			return 'failed'

		self.mgc.execute(traj)


		print "moved lin to " + str(param)
		return 'succeeded'

### Helper function 
def gen_pose(frame_id="/base_link", pos=[0,0,0], euler=[0,0,0]):
	pose = PoseStamped()
	pose.header.frame_id = frame_id
	pose.header.stamp = rospy.Time.now()
	pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pos
	pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_euler(*euler)
	return pose
