##############################
#      Message to define     #
#  one pose on a trajectory  #
##############################

#Frame this data is associated with
# 0: no frame
# 1: global frame (robot base)
string frame_id

#pose
geometry_msgs/Pose pose

#velocity of linear movement m/s
float32 linVel

#acceleration of linear movement m/s^2
float32 linAcc

#movement type (ptp = 0, lin = 1)
int32 movement_type

#blend radius mm (0 = no blending/no continous movement)
float32 blend_radius
