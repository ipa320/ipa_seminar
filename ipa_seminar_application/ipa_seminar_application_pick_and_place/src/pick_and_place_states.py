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
from simple_script_server import *

### Create a handle for the Simple Script Server Interface
sss = simple_script_server()
### Create a handle for the Planning Scene Interface
psi = PlanningSceneInterface()
### Create a handle for the Move Group Commander
mgc = MoveGroupCommander("arm_gripper")

class prepare_robot(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])
		self.og = open_gripper()

	def execute(self, userdata):
		print "preparing robot"
		self.og.execute(userdata)
		print "robot prepared"
		return 'succeeded'

class move_planned(smach.State):
	def __init__(self, pose_stamped):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])
		self.pose_stamped = pose_stamped

	def execute(self, userdata):
		# plan trajectory
		traj = mgc.plan(self.pose_stamped.pose)
		if len(traj.joint_trajectory.points) == 0: # TODO is there a better way to know if planning failed or not?
			return 'failed'
		
		# execute trajectory
		mgc.execute(traj)

		return 'succeeded'

class move_lin(smach.State):
	def __init__(self, pose_stamped):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])
		self.pose_stamped = pose_stamped

	def execute(self, userdata):
		# plan trajectory
		(traj,frac) = mgc.compute_cartesian_path([self.pose_stamped.pose], 0.01, 4, False)
		if len(traj.joint_trajectory.points) == 0: # TODO is there a better way to know if planning failed or not?
			return 'failed'
		if frac != 1.0: # this means moveit couldn't plan to the end
			return 'failed'

		# execute trajectory
		mgc.execute(traj)

		return 'succeeded'

class open_gripper(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])
		self.service_name = "/MoveGripper"
		self.client = rospy.ServiceProxy(self.service_name, MoveGripper)

	def execute(self, userdata):
		print "open gripper"

		try:		
			# check if gripper service is available
			rospy.wait_for_service(self.service_name, 5)

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'

		# move gripper
		if rospy.has_param("/use_sim_time"):
			sss.move("gripper","open")
		else:
			try:		
				self.client(1)
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			rospy.sleep(8)

		print "gripper opened"
		return 'succeeded'

class close_gripper(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])
		self.service_name = "/MoveGripper"
		self.client = rospy.ServiceProxy(self.service_name, MoveGripper)

	def execute(self, userdata):
		print "close gripper"

		try:		
			# check if gripper service is available
			rospy.wait_for_service(self.service_name, 5)

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'
	
		# move gripper
		if rospy.has_param("/use_sim_time"):
			sss.move("gripper","close")
		else:
			try:
				self.client(0)
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			rospy.sleep(8)

		print "gripper closed"
		return 'succeeded'


### sub state machines
class pick_object(smach.StateMachine):
	def __init__(self, area):	
		smach.StateMachine.__init__(self, 
			outcomes=['object_picked', 'object_not_picked', 'error'])
		
		# calculate poses
		self.box_pose = get_pose_from_parameter_server(area)
		self.down_pose = copy.deepcopy(self.box_pose)
		self.down_pose.pose.position.z += 0.1
		self.up_pose = copy.deepcopy(self.down_pose)
		self.up_pose.pose.position.z += 0.1

		with self:
			smach.StateMachine.add('MOVE_TO_PICK_UP_POSITION1', move_planned(self.up_pose),
				transitions={'succeeded':'MOVE_TO_PICK_DOWN_POSITION', 
							'failed':'object_not_picked'})

			smach.StateMachine.add('MOVE_TO_PICK_DOWN_POSITION', move_lin(self.down_pose),
				transitions={'succeeded':'CLOSE_GRIPPER',
							'failed':'object_not_picked'})

			smach.StateMachine.add('CLOSE_GRIPPER', close_gripper(),
				transitions={'succeeded':'MOVE_TO_PICK_UP_POSITION2', 
							'failed':'object_not_picked'})

			smach.StateMachine.add('MOVE_TO_PICK_UP_POSITION2', move_lin(self.up_pose),
				transitions={'succeeded':'object_picked', 
							'failed':'object_not_picked'})

class place_object(smach.StateMachine):
	def __init__(self, area):	
		smach.StateMachine.__init__(self, 
			outcomes=['object_placed', 'object_not_placed', 'error'])

		# calculate poses
		self.box_pose = get_pose_from_parameter_server(area)
		self.down_pose = copy.deepcopy(self.box_pose)
		self.down_pose.pose.position.z += 0.1
		self.up_pose = copy.deepcopy(self.down_pose)
		self.up_pose.pose.position.z += 0.1

		with self:
			smach.StateMachine.add('MOVE_TO_PLACE_UP_POSITION1', move_planned(self.up_pose),
				transitions={'succeeded':'MOVE_TO_PLACE_DOWN_POSITION', 
							'failed':'object_not_placed',
							'error':'error'})

			smach.StateMachine.add('MOVE_TO_PLACE_DOWN_POSITION', move_lin(self.down_pose),
				transitions={'succeeded':'OPEN_GRIPPER',
							'failed':'object_not_placed',
							'error':'error'})

			smach.StateMachine.add('OPEN_GRIPPER', open_gripper(),
				transitions={'succeeded':'MOVE_TO_PLACE_UP_POSITION2', 
							'failed':'object_not_placed',
							'error':'error'})

			smach.StateMachine.add('MOVE_TO_PLACE_UP_POSITION2', move_lin(self.up_pose),
				transitions={'succeeded':'object_placed', 
							'failed':'object_not_placed',
							'error':'error'})



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
	param_string = "/application/" + param_name
	if not rospy.has_param(param_string):
		rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",param_string)
		return 'failed'
	param = rospy.get_param(param_string)

	# transform box position from arm_0_link to base_link
	param[0][2] += 1.1 # lift along z-axis 

	# fill pose message
	return gen_pose(pos = param[0], euler = param[1])
