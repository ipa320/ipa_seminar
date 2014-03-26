##############################
#      Message to define     #
#  one pose on a trajectory  #
##############################

#Frame this data is associated with
# 0: no frame
# 1: global frame (robot base)
# 2: flange
# 3: tool-tip
string frame_id

#identifier for feedback
string pose_id

#type
# 0: cartesian (default)
# 1: joint positions
string poseType

#pose
#geometry_msgs/Pose pose
# alternative: instead of Euler, use joint-angles but remember to define "poseType" accordingly
float32[] pose #[X,Y,Z,A,B,C] (Euler ZYX)

#velocity of linear movement m/s
float32 linVel

#acceleration of linear movement m/s^2
float32 linAcc

#movement type (ptp = 0, lin = 1)
int32 movement_type

#blend radius mm (0 = no blending/no continous movement)
float32 blend_radius
