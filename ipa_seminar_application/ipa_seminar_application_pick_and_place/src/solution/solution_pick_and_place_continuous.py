#!/usr/bin/python

import roslib; roslib.load_manifest('ipa_seminar_application_pick_and_place')
import rospy

import smach
import smach_ros

# import scenario specific states and (sub-)state machines
from pick_and_place_states import *

class pick_and_place_object(smach.StateMachine):
	def __init__(self, source_area, target_area):	
		smach.StateMachine.__init__(self, 
			outcomes=['succeeded', 'failed', 'error'])

		with self:
			smach.StateMachine.add('PICK_OBJECT', pick_object(source_area),
				transitions={'object_picked':'PLACE_OBJECT', 
							'object_not_picked':'failed',	
							'error':'error'})

			smach.StateMachine.add('PLACE_OBJECT', place_object(target_area),
				transitions={'object_placed':'PICK_OBJECT',
							'object_not_placed':'failed',
							'error':'error'})

# main
def main():
	rospy.init_node('application')

	# create a SMACH state machine
	SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed', 'error'])

	# open the container
	with SM:
		smach.StateMachine.add('PREPARE_ROBOT', prepare_robot(),
			transitions={	'succeeded':'PICK_AND_PLACE_OBJECT',
							'failed':'overall_failed'})

		smach.StateMachine.add('PICK_AND_PLACE_OBJECT', pick_and_place_object(source_area="area_1", target_area="area_2"),
			transitions={	'succeeded':'overall_succeeded',
							'failed':'overall_failed',
							'error':'error'})

	# Start SMACH viewer
	smach_viewer = smach_ros.IntrospectionServer('PICK_AND_PLACE', SM, 'PICK_AND_PLACE')
	smach_viewer.start()

	SM.execute()

	# stop SMACH viewer
	rospy.spin()
	# smach_thread.stop()
	smach_viewer.stop()

if __name__ == '__main__':
	main()
