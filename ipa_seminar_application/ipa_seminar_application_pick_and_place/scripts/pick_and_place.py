#!/usr/bin/python

import roslib; roslib.load_manifest('ipa_seminar_application_pick_and_place')
import rospy

import smach
import smach_ros

# scenario specific states and (sub-)state machines
from pick_and_place_states import *

# main
def main():
	rospy.init_node('ipa_seminar_application_pick_and_place')

	# create a SMACH state machine
	SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed'])

	#parameters in yaml file
	# pick_offset_z

	# open the container
	with SM:
		# add states to the container
		smach.StateMachine.add('TEST1', move_ptp("pick_position"),
			transitions={	'succeeded':'TEST2',
							'failed':'overall_failed'})

		smach.StateMachine.add('TEST2', move_lin("pick2_position"),
			transitions={	'succeeded':'TEST1',
							'failed':'overall_failed'})

#		smach.StateMachine.add('PREPARE_ROBOT', prepare_robot(),
#			transitions={	'succeeded':'MOVE_TO_PICK_POSITION',
#							'failed':'overall_failed'})
#
#		smach.StateMachine.add('MOVE_TO_PICK_POSITION', move_ptp("pick_position"),
#			transitions={	'succeeded':'PICK_OBJECT',
#							'failed':'overall_failed'})
#
#		smach.StateMachine.add('PICK_OBJECT', pick_object(),
#			transitions={	'object_picked':'MOVE_TO_PLACE_POSITION',
#							'object_not_picked':'MOVE_TO_PICK_POSITION',
#							'failed':'overall_failed'})
#		
#		smach.StateMachine.add('MOVE_TO_PLACE_POSITION', move_ptp("place_position"),
#			transitions={	'succeeded':'PLACE_OBJECT',
#							'failed':'overall_failed'})
#
#		smach.StateMachine.add('PLACE_OBJECT', place_object(),
#			transitions={	'object_placed':'MOVE_TO_HOME_POSITION',
#							'object_not_placed':'MOVE_TO_PLACE_POSITION',
#							'failed':'overall_failed'})
#
#		smach.StateMachine.add('MOVE_TO_HOME_POSITION', move_ptp("home"),
#			transitions={	'succeeded':'overall_succeeded', 
#							'failed':'overall_failed'})

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
