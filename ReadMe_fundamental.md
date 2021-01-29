# NOte
iris, iris_asus are in sitl mode. If you want to do hitl, please modify sdf.
# install
First we recommand you follow https://www.yuque.com/xtdrone/manual_cn/basic_config_1.11 strictly!
 to install dependencies. Run one demo before trying below. 
Then you replace the official repo with this one.
```
git clone  https://gitee.com/jinxer000/fundamental_sys_xtdrone.git
git submodule update --init --recursive
```
One additional step: 
```
  cp -r sitl_config/models/*  ~/.gazebo/models/
  ```
Also make sure cpc_aux_mapping, cpc_motion_planning, cpc_reference_publisher,cpc_ws in imav branch.
Then compile two repos respectively.

You can also copy the catkin_ws into your home folder. 
# Use simulator without offboard
```
cd PX4_Firmware
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 indoor1.launch 

cd control_scripts
python multirotor_communication_manual.py iris 0
python multirotor_keyboard_control.py iris 1 vel

```
Then you might control the UAV manually.
If you want to use 3d lidar, please replace line 15 with line 14 in indoor1.launch.
# indoor demo with offboard
```
cd PX4_Firmware
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 indoor1.launch 

cd control_scripts
python multirotor_communication.py iris 0

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
## developing
roslaunch cpc_aux_mapping laser3d_sim.launch
roslaunch cpc_aux_mapping sim_uav.launch

launch-prefix="xterm -e cuda-gdb --args"

