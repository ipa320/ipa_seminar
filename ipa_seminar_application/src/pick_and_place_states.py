#!/usr/bin/python

import roslib; roslib.load_manifest('ipa_seminar_application')
import rospy

import smach
import smach_ros

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

	def execute(self, userdata):
		print "picking object"
		rospy.sleep(3)
		print "object picked"
		return 'object_picked'

class place_object(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['object_placed', 'object_not_placed', 'failed'])

	def execute(self, userdata):
		print "placing object"
		rospy.sleep(3)
		print "object placed"
		return 'object_placed'

class move_ptp(smach.State):
	def __init__(self, position):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed'])
		self.position = position

	def execute(self, userdata):
		print "move ptp to " + self.position
		rospy.sleep(3)
		print "object picked"
		return 'succeeded'
