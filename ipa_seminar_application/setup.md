disable sticky edges, check monitor configuration


add users (usernames: group1...group4, pw: group1...group4)
```
sudo adduser group1

```


on ros account
```
sudo apt-get install ros-groovy-cob* ros-groovy-moveit-*
mkdir ~/git
rosinstall -j 8 ~/git https://raw.github.com/ipa320/setup/master/repositories/seminar_application.rosinstall
sudo rosdep init
rosdep update
```


on all user accounts
```
mkdir ~/git
git clone /home/ros/git/ipa_seminar ~/git/ipa_seminar
rosdep update
```


add to ~/.bashrc:
```
source /opt/ros/groovy/setup.bash
export ROS_PACKAGE_PATH=~/git:/home/ros/git:$ROS_PACKAGE_PATH

export ROS_PARALLEL_JOBS=-j5

#show ROS_MASTER_URI in bash prompt
STARTCOLOR='\e[0;34m';
ENDCOLOR="\e[0m"
export PS1="$STARTCOLOR[\u@\h\w>] \e[0;32m[{$ROS_MASTER_URI}] $ENDCOLOR\$ "
```

build
```
rosmake ipa_seminar_application_bringup ipa_seminar_application_pick_and_place
```

update user accounts
```
cd ~/git/ipa_seminar
git pull origin master
```
