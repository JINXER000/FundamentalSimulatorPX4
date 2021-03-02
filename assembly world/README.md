Environment: XTdrone (PX4-1.11.0 + gazebo9) + gazebo_ros_link_attacher (get it on Github)

1. First put all the packages into sitl_gazebo:
cp -r models/* ~/PX4_Firmware/Tools/sitl_gazebo/models
cp -r worlds/* ~/PX4_Firmware/Tools/sitl_gazebo/worlds
cp -r launch/* ~/PX4_Firmware/launch
cp -r gazebo_amu_motor_model.cpp ~/PX4_Firmware/Tools/sitl_gazebo/src
cd ~/PX4_Firmware
make px4_sitl_default gazebo

2. Now you can run these launch file by:
roslaunch px4 amu.launch
roslaunch px4 amu_rope.launch
roslaunch px4 amu_cooperative_assembly.launch
roslaunch px4 iris_cooperative_assembly.launch
roslaunch px4 p700.launch
roslaunch px4 p700_rope.launch
roslaunch px4 p700_cooperative_assembly.launch


3. Then you need to install gazebo_ros_link_attacher plugin following this github instructions:
https://github.com/pal-robotics/gazebo_ros_link_attacher

4. After this, you need to put this plugin into ~/.bashrc

gedit ~/.bashrc
and put the following line into it:
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_ros_link_attacher/devel/lib

5. Finally, to run this, 

In one terminal, open one cooperative_assembly launch, for instance:
roslaunch px4 amu_cooperative_assembly.launch

In another terminal: copy the "cooperative_attach.py" into ~/gazebo_ros_link_attacker/scripts
source ~/gazebo_ros_link_attacher/devel/setup.bash

python ~/gazebo_ros_link_attacher/src/gazebo_ros_link_attacher/scripts/cooperative_attach.py



you will see:
[INFO] : Attaching iris_0 and payload
[INFO] : Attaching iris_1 and payload
[INFO] : Attaching iris_2 and payload
It indicates UAVs and payload are linked together

Then:
cd ~/XTDrone/communication
python multirotor_communication_manual.py iris 0
python multirotor_communication_manual.py iris 1
python multirotor_communication_manual.py iris 2

cd ~/XTDrone/control
python multirotor_keyboard_control.py iris 3 vel

After taking off, you need to delete the shelfs:
rosservice call /gazebo/delete_model "model_name: 'shelfs'"
