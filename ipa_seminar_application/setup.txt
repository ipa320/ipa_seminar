mkdir ~/git
rosinstall -j 8 ~/git application.rosinstall



add to ~/.bashrc:
```
source /opt/ros/groovy/setup.bash
export ROS_PACKAGE_PATH=~/git/ipa_seminar:$ROS_PACKAGE_PATH

export ROS_PARALLEL_JOBS=-j5

#show ROS_MASTER_URI in bash prompt
export PS1="\u@\h:\w\e[0;32m{$ROS_MASTER_URI}\e[m$"
```
