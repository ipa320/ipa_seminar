#!/usr/bin/python

import roslib; roslib.load_manifest('ipa_seminar_application_pick_and_place')
import rospy
import copy

import smach
import smach_ros

from tf.transformations import *
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from brics_showcase_industry_interfaces.srv import *

class prepare_robot(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed'])

	def execute(self, userdata):
		print "preparing robot"
		rospy.sleep(3)
		print "robot prepared"
		return 'succeeded'

class move_planned(smach.State):
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
		(traj,frac) = self.mgc.compute_cartesian_path([goal_pose.pose], 0.01, 4, False)
		if len(traj.joint_trajectory.points) == 0: # TODO is there a better way to know if planning failed or not?
			return 'failed'
		if frac != 1.0: # this means moveit couldn't plan to the end
			return 'failed'

		# execute trajectory
		self.mgc.execute(traj)

		print "moved lin to " + str(self.position)
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
		(traj,frac) = self.mgc.compute_cartesian_path([goal_pose.pose], 0.01, 4, False)
		if len(traj.joint_trajectory.points) == 0: # TODO is there a better way to know if planning failed or not?
			return 'failed'
		
		# execute trajectory
		self.mgc.execute(traj)

		print "moved lin to " + str(self.position)
		return 'succeeded'

class open_gripper(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed'])
		self.service_name = "/MoveGripper"
		self.client = rospy.ServiceProxy(self.service_name, MoveGripper)

	def execute(self, userdata):
		print "open gripper"

		try:		
			# check if gripper service is available
			rospy.wait_for_service(self.service_name, 5)

			# move gripper
			self.client(1)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'

		print "gripper opened"
		return 'succeeded'

class close_gripper(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed'])
		self.service_name = "/MoveGripper"
		self.client = rospy.ServiceProxy(self.service_name, MoveGripper)

	def execute(self, userdata):
		print "close gripper"

		try:		
			# check if gripper service is available
			rospy.wait_for_service(self.service_name, 5)

			# move gripper
			self.client(0)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'

		print "gripper closed"
		return 'succeeded'


### sub state machines
class pick_object(smach.StateMachine):
	def __init__(self, prefix):	
		smach.StateMachine.__init__(self, 
			outcomes=['object_picked', 'object_not_picked', 'failed'])
		self.prefix = prefix

		with self:
			smach.StateMachine.add('MOVE_TO_PICK_UP_POSITION1', move_planned(prefix + "up_position"),
				transitions={'succeeded':'MOVE_TO_PICK_DOWN_POSITION', 
							'failed':'object_not_picked'})

			smach.StateMachine.add('MOVE_TO_PICK_DOWN_POSITION', move_lin(prefix + "down_position"),
				transitions={'succeeded':'CLOSE_GRIPPER',
							'failed':'object_not_picked'})

			smach.StateMachine.add('CLOSE_GRIPPER', close_gripper(),
				transitions={'succeeded':'MOVE_TO_PICK_UP_POSITION2', 
							'failed':'object_not_picked'})

			smach.StateMachine.add('MOVE_TO_PICK_UP_POSITION2', move_lin(prefix + "up_position"),
				transitions={'succeeded':'object_picked', 
							'failed':'object_not_picked'})

class place_object(smach.StateMachine):
	def __init__(self, prefix):	
		smach.StateMachine.__init__(self, 
			outcomes=['object_placed', 'object_not_placed', 'failed'])
		self.prefix = prefix

		with self:
			smach.StateMachine.add('MOVE_TO_PLACE_UP_POSITION1', move_planned(prefix + "up_position"),
				transitions={'succeeded':'MOVE_TO_PLACE_DOWN_POSITION', 
							'failed':'object_not_placed'})

			smach.StateMachine.add('MOVE_TO_PLACE_DOWN_POSITION', move_lin(prefix + "down_position"),
				transitions={'succeeded':'OPEN_GRIPPER',
							'failed':'object_not_placed'})

			smach.StateMachine.add('OPEN_GRIPPER', open_gripper(),
				transitions={'succeeded':'MOVE_TO_PLACE_UP_POSITION2', 
							'failed':'object_not_placed'})

			smach.StateMachine.add('MOVE_TO_PLACE_UP_POSITION2', move_lin(prefix + "up_position"),
				transitions={'succeeded':'object_placed', 
							'failed':'object_not_placed'})



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
