# robo_arm
A ROS Powered Robot Arm project


Put gripper to 90 degrees, then put it in open position at 90. Then as you increase angle, grip will tighten.

First, make a catkin workspace for the robo arm.
```
mkdir -p ~/robo_arm_catkin/src
cd ~/robo_arm_catkin
catkin_make
```

Then, go back to your home and copy the repo.
```
cd ~
git clone https://github.com/MZandtheRaspberryPi/robo_arm.git
```

And move the packages from this repo to your catkin ws.
```
cp -r ~/robo_arm/* ~/robo_arm_catkin/src/
```

Now you can make the catkin ws and then source it and run commands.
```
cd ~/robo_arm_catkin
catkin_make
source ~/robo_arm_catkin/devel/setup.bash
```

To generate the custom arduino messages, find the location of your arduino libraries folder and remove the ros_lib, and regenerate it. Make sure you've sourced the robo_arm workspace.
```
rm -rf /home/mz/Arduino/libraries/ros_lib
rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries
```

Running the launch file. Note that mine is set to ttyS0 as i'm using a VM and passing the com port to the VM. If you have your robo arm connected to a different serial port you'll need to edit the launch file and change the parameter for the serial port accordingly.
```
roslaunch robo_arm robo_arm.launch 
```

Early on, realized that having 1 publisher and 1 subscriber in an Arduino Uno was pushing memory. Hence, trimmed down message with just changes in x, y, z, and gripper, and time to reach there.

message:
```
rostopic pub /robo_arm_teleop/arm_movementobo_arm/arm_control -1 '{x: 0, y: 180, z: 158, grip: 1700, duration: 1000}'
```

