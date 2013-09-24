#!/usr/bin/python

import roslib; roslib.load_manifest('ipa_seminar_application_pick_and_place')
import rospy
import copy

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
		
		# get joint_names from parameter server
		param_string = "/pick_and_place/pick_offset"
		if not rospy.has_param(param_string):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_string)
			return 'failed'
		pick_offset = rospy.get_param(param_string)
	
		# plan trajectory
		goal_pose_down = get_pose_from_parameter_server("pick_position")
		goal_pose_up = copy.deepcopy(goal_pose_down)
		goal_pose_up.pose.position.z += pick_offset

		### Move_lin down
		(traj,frac) = self.mgc.compute_cartesian_path([goal_pose_up.pose, goal_pose_down.pose], 0.01, 4, True)
		if len(traj.joint_trajectory.points) == 0: # TODO is there a better way to know if planning failed or not?
			return 'failed'
		self.mgc.execute(traj)

		# close gripper
		#TODO
		print "close gripper"
		rospy.sleep(3)
		
		### Move_lin up
		(traj,frac) = self.mgc.compute_cartesian_path([goal_pose_up.pose], 0.01, 4, True)
		if len(traj.joint_trajectory.points) == 0: # TODO is there a better way to know if planning failed or not?
			return 'failed'
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

		# get joint_names from parameter server
		param_string = "/pick_and_place/place_offset"
		if not rospy.has_param(param_string):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_string)
			return 'failed'
		pick_offset = rospy.get_param(param_string)
	
		# plan trajectory
		goal_pose_down = get_pose_from_parameter_server("place_position")
		goal_pose_up = copy.deepcopy(goal_pose_down)
		goal_pose_up.pose.position.z += pick_offset

		### Move_lin down
		(traj,frac) = self.mgc.compute_cartesian_path([goal_pose_up.pose, goal_pose_down.pose], 0.01, 4, True)
		if len(traj.joint_trajectory.points) == 0: # TODO is there a better way to know if planning failed or not?
			return 'failed'
		self.mgc.execute(traj)

		# open gripper
		#TODO
		print "open gripper"
		rospy.sleep(3)
		
		### Move_lin up
		(traj,frac) = self.mgc.compute_cartesian_path([goal_pose_up.pose], 0.01, 4, True)
		if len(traj.joint_trajectory.points) == 0: # TODO is there a better way to know if planning failed or not?
			return 'failed'
		self.mgc.execute(traj)

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

		# plan trajectory
		goal_pose = get_pose_from_parameter_server(self.position)
		traj = self.mgc.plan(goal_pose.pose)
		if len(traj.joint_trajectory.points) == 0: # TODO is there a better way to know if planning failed or not?
			return 'failed'
		
		# execute trajectory
		self.mgc.execute(traj)

		print "moved ptp to " + str(self.position)
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

		# plan trajectory
		goal_pose = get_pose_from_parameter_server(self.position)
		(traj,frac) = self.mgc.compute_cartesian_path([goal_pose.pose], 0.01, 4, True)
		if len(traj.joint_trajectory.points) == 0: # TODO is there a better way to know if planning failed or not?
			return 'failed'
		
		# execute trajectory
		self.mgc.execute(traj)

		print "moved lin to " + str(self.position)
		return 'succeeded'




### Helper function 
def gen_pose(frame_id="/base_link", pos=[0,0,0], euler=[0,0,0]):
	pose = PoseStamped()
	pose.header.frame_id = frame_id
	pose.header.stamp = rospy.Time.now()
	pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pos
	pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_euler(*euler)
	return pose

def get_pose_from_parameter_server(param_name):
	# get joint_names from parameter server
	param_string = "/pick_and_place/" + param_name
	if not rospy.has_param(param_string):
		rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_string)
		return 'failed'
	param = rospy.get_param(param_string)

	# fill pose message
	return gen_pose(pos = param[0], euler = param[1])
