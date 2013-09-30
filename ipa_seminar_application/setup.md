disable sticky edges


add users (usernames: group1...group4, pw: group1...group4)
```
sudo adduser group1

```



```
sudo apt-get install ros-groovy-cob* ros-groovy-moveit-*

mkdir ~/git
rosinstall -j 8 ~/git application.rosinstall
```


add to ~/.bashrc:
```
source /opt/ros/groovy/setup.bash
export ROS_PACKAGE_PATH=~/git:$ROS_PACKAGE_PATH

export ROS_PARALLEL_JOBS=-j5

#show ROS_MASTER_URI in bash prompt
export PS1="\u@\h:\w\e[0;32m{$ROS_MASTER_URI}\e[m$"
```
