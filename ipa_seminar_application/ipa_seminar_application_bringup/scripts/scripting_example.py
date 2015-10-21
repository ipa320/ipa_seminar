#!/usr/bin/env python
import roslib; roslib.load_manifest('lbr_bringup')
import rospy

from tf.transformations import *
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
 
### Helper function 
def gen_pose(frame_id="/base_link", pos=[0,0,0], euler=[0,0,0]):
	pose = PoseStamped()
	pose.header.frame_id = frame_id
	pose.header.stamp = rospy.Time.now()
	pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pos
	pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quaternion_from_euler(*euler)
	return pose

### fix trajectory
def fix_trajectory(traj, speed_factor = 1.0):
	for i in range(len(traj.joint_trajectory.points)):
		traj.joint_trajectory.points[i].velocities = [0]*len(traj.joint_trajectory.points[i].positions)
		traj.joint_trajectory.points[i].time_from_start *= 1.0/speed_factor
	return traj

if __name__ == '__main__':
	rospy.init_node('scripting_example')
	while rospy.get_time() == 0.0: pass
	
	### Create a handle for the Planning Scene Interface
	psi = PlanningSceneInterface()
	rospy.sleep(1.0)
	
	### Create a handle for the Move Group Commander
	mgc = MoveGroupCommander("arm")
	rospy.sleep(1.0)
	
	
	### Add virtual obstacle
	pose = gen_pose(pos=[0.5, -0.2, 1.2])
	psi.add_box("box", pose, size=(0.15, 0.15, 0.5))
	rospy.sleep(1.0)

	### Move to joint goal
	mgc.go([-1.3780676708976536, -1.6994479592734537, 1.3636316141915703, -2.647661781458232, -2.213850085030142, 1.6799727955218422])

	### Move to stored joint position
	mgc.set_named_target("right")
	mgc.go()
	
	### Move to Cartesian position
	goal_pose = gen_pose(frame_id='/base_link', pos=[0.573, 0.292, 1.326], euler=[-1.559, 0.003, -2.737])
	mgc.go(goal_pose.pose)
	
	### Move Cartesian linear
	goal_pose.pose.position.z -= 0.1
	(traj,frac) = mgc.compute_cartesian_path([goal_pose.pose], 0.01, 4, True)
	traj_exec = fix_trajectory(traj)

	mgc.execute(traj)
	
	print "Done"
