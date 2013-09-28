#!/usr/bin/python

import roslib; roslib.load_manifest('ipa_seminar_application_pick_and_place')
import rospy

import smach
import smach_ros

# import scenario specific states and (sub-)state machines
from pick_and_place_states import *
from pick_and_place import *

class switch_objects(smach.StateMachine):
    def __init__(self, area_a, area_b, buffer_area):	
        smach.StateMachine.__init__(self, 
            outcomes=['succeeded', 'failed', 'error'])

        with self:
            smach.StateMachine.add('A_TO_BUFFER', pick_and_place_object(area_a, buffer_area),
                transitions={'succeeded':'B_TO_A', 
                    'failed':'failed',
                    'error':'error'})

            smach.StateMachine.add('B_TO_A', pick_and_place_object(area_b, area_a),
                transitions={'succeeded':'BUFFER_TO_B',
                'failed':'failed',
                'error':'error'})

            smach.StateMachine.add('BUFFER_TO_B', pick_and_place_object(buffer_area, area_b),
                transitions={'succeeded':'succeeded',
                    'failed':'failed',
                    'error':'error'})

# main
def main():
	rospy.init_node('application')

	# create a SMACH state machine
	SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed', 'error'])

	# open the container
	with SM:
		smach.StateMachine.add('PREPARE_ROBOT', prepare_robot(),
			transitions={	'succeeded':'SWITCH_OBJECTS',
							'failed':'overall_failed'})

		smach.StateMachine.add('SWITCH_OBJECTS', switch_objects(area_a="area_1", area_b="area_2", buffer_area="area_5"),
			transitions={	'succeeded':'overall_succeeded',
							'failed':'overall_failed',
							'error':'error'})

	# Start SMACH viewer
	smach_viewer = smach_ros.IntrospectionServer('SWITCH', SM, 'SWITCH')
	smach_viewer.start()

	SM.execute()

	# stop SMACH viewer
	rospy.spin()
	# smach_thread.stop()
	smach_viewer.stop()

if __name__ == '__main__':
	main()
