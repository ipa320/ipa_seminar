#!/usr/bin/env bash

### ROS
source /opt/ros/groovy/setup.bash
#source /home/ros/git/care-o-bot_catkin/devel/setup.bash
export ROS_PACKAGE_PATH=/home/ros/git/care-o-bot:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=~/git/ipa_seminar:$ROS_PACKAGE_PATH

export ROBOT=lbr_solo
export ROBOT_ENV=empty
export ROS_MASTER_URI=http://localhost:11311

alias solution='source ~/.bashrc && export ROS_PACKAGE_PATH=/home/ros/git/ipa_seminar:$ROS_PACKAGE_PATH'

alias ssh-robot='ssh -X ros@pyramid-2'
alias exp-robot='export ROS_MASTER_URI=http://pyramid-2:11311'
