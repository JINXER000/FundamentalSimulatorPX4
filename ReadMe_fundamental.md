# NOte
- iris, iris_asus are in sitl mode. If you want to do hitl, please modify sdf.
- after using git pull, do 'git submodule  init & git submodule update'
# install
## basic requirements: 
ubuntu 18.04, gazebo 9.10+
## steps
First we recommand you follow https://www.yuque.com/xtdrone/manual_cn/basic_config_1.11 strictly!
 to install dependencies. Run one demo before trying below. 
Then you replace the official repo with this one.
```
git clone  https://gitee.com/jinxer000/fundamental_sys_xtdrone.git
git submodule update --init --recursive
```
One additional step: 
```
  cp -r Tools/sitl_gazebo/models/*  ~/.gazebo/models/
  ```
- add human model:
```
cp -r Tools/sitl_gazebo/models/ihuman/* ~/.gazebo/models/
```
Also make sure cpc_aux_mapping, cpc_motion_planning, cpc_reference_publisher,cpc_ws in imav branch.
Then compile two repos respectively.
For PX4_firmware, you can do
```
git tag -a v1.11.0 -m "init"
make px4_sitl_default gazebo
```


You can also copy the catkin_ws into your home folder. 

## dummy update
cp ~/PX4_Firmware/Tools/sitl_gazebo/models/ihumanv1/* ~/.gazebo/models/
# Use simulator without offboard

```
cd PX4_Firmware
cp -r catkin_ws/* ~/
source ~/catkin_ws/devel/setup.bash   
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 imav_indoor_rich.launch 

cd control_scripts
python multirotor_communication_manual.py iris 0
python multirotor_keyboard_control.py iris 1 vel

```
Then you might control the UAV manually.
If you want to use 3d lidar, please replace line 15 with line 14 in launch file
# indoor demo with offboard
```
cd PX4_Firmware
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 imav_indoor_rich.launch 

cd control_scripts
python multirotor_communication.py iris 0
rosservice call /iris_0/engage

cd cpc_ws
roslaunch cpc_aux_mapping hitl_sim.launch
roslaunch cpc_motion_planning motion_nf1_sitl.launch

roslaunch mobile_cylinder multi_box.launch
```
Then use 2d navigation goal to set target
# multi-agent demo with offboard
Since the target is same, agents will crash with each other.
```
cd PX4_Firmware
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4  multi_vehicle.launch

cd control_scripts
python multirotor_communication.py iris 0
python multirotor_communication.py iris 1
rosservice call /iris_0/engage
rosservice call /iris_1/engage

cd cpc_ws
roslaunch cpc_aux_mapping hitl_multi.launch
roslaunch cpc_motion_planning motion_hitl_multi.launch

```
## HITL mode

First, follow https://docs.px4.io/master/en/simulation/hitl.html and do a demo with RC.
After modifying sdf, turn off RC.


```
cd PX4_Firmware
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 hitl.launch 

cd control_scripts
python multirotor_communication.py iris 0
rosservice call /iris_0/engage
```
You will see the UAV takes off. 
Then
```
cd PX4_Firmware
cp launch/mavros_hitl.launch ~/catkin_ws/src/mavros/launch/
cd ~/catkin_ws/
roslaunch mavros mavros_hitl.launch

cd cpc_ws
roslaunch cpc_aux_mapping hitl_sim.launch
roslaunch cpc_motion_planning motion_nf1_sitl.launch

roslaunch mobile_cylinder multi_box.launch
```
Then you can set target in rviz!

## cooperative 
you need to install gazebo_ros_link_attacher plugin following this github instructions:
https://github.com/pal-robotics/gazebo_ros_link_attacher
```
cd PX4_Firmware
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_ros_link_attacher/devel/lib
roslaunch px4 amu_cooperative_assembly.launch
```
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

## developing

roslaunch cpc_aux_mapping laser3d_sim.launch
roslaunch cpc_aux_mapping sim_uav.launch

launch-prefix="xterm -e cuda-gdb --args"

## about texture (For Zhaiyu)
Scripts can be found in:
/usr/share/gazebo-9/media/materials/scripts$

But in this repo,  you can check Tools/sitl_gazebo/models/rover/material/   instead.


