#!/usr/bin/env python
import roslib
roslib.load_manifest('ipa_seminar_application_bringup')
roslib.load_manifest('ipa_seminar_application_pick_and_place')
import rospy
import copy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from pick_and_place_states import get_pose_from_parameter_server

def publisher():
	rospy.init_node('area_publisher')
	pub = rospy.Publisher('areas', Marker)

	while not rospy.is_shutdown():
		try:
			color = ColorRGBA()
			color.r = 0
			color.g = 1
			color.b = 0
			color.a = 1 
			areas = rospy.get_param("/application")

			for area in areas:
				# cube
				marker = Marker()
				marker.header.frame_id = "/base_link"
				marker.header.stamp = rospy.Time.now()
				marker.pose = get_pose_from_parameter_server(area).pose
				marker.pose.position.z = 1.0
				marker.ns = area + "cube"
				marker.type = 1
				marker.scale.x = 0.1
				marker.scale.y = 0.1
				marker.scale.z = 0.01
				marker.color = color
				pub.publish(marker)

				# arrow
				arrow_marker = Marker()
				arrow_marker.header.frame_id = "/base_link"
				arrow_marker.header.stamp = rospy.Time.now()
				arrow_marker.pose = get_pose_from_parameter_server(area).pose
				arrow_marker.pose.position.z = 1.01
				arrow_marker.ns = area + "arrow"
				arrow_marker.type = 0
				arrow_marker.scale.x = 0.2
				arrow_marker.scale.y = 0.01
				#arrow_marker.scale.z = 0.3
				arrow_marker.color.r = 1
				arrow_marker.color.g = 0
				arrow_marker.color.b = 0
				arrow_marker.color.a = 1
				pub.publish(arrow_marker)

				# text
				text_marker = Marker()
				text_marker.header.frame_id = "/base_link"
				text_marker.header.stamp = rospy.Time.now()
				text_marker.pose = get_pose_from_parameter_server(area).pose
				text_marker.pose.position.z = 1.05
				text_marker.ns = area + "text"
				text_marker.type = 9
				text_marker.scale.z = 0.075
				text_marker.color = color
				text_marker.text = area
				pub.publish(text_marker)

				# change color for next marker
				color.g -= 0.1

			rospy.sleep(1)
		except:
			pass

if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
