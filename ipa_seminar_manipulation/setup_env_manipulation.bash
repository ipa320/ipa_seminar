#!/usr/bin/env bash

### ROS
#source /opt/ros/indigo/setup.bash
#source /home/ros/seminar/devel/setup.bash
source ~/seminar/devel/setup.bash

export ROBOT=lbr_solo
export ROBOT_ENV=empty
export ROS_MASTER_URI=http://localhost:11311

alias group='source ~/seminar/devel/setup.bash'
alias solution='source /home/ros/seminar/devel/setup.bash'

alias ssh-robot='ssh -X ros@pyramid-2'
alias exp-robot='export ROS_MASTER_URI=http://pyramid-2:11311'
