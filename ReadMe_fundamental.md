# Note
- iris, iris_asus are in sitl mode. If you want to do hitl, please modify sdf.
- revise asus: <pointCloudCutoffMax>=10.0
- revise 3d_lidar: <vertical><samples>=16
# install
## basic requirements: 
ubuntu 18.04, gazebo 9.10+
## steps
First we recommand you follow https://www.yuque.com/xtdrone/manual_cn/basic_config_1.11 strictly!
 to install dependencies. Run one demo before trying below. 

Then you can download our repo:
```
git clone  https://gitee.com/jinxer000/fundamental_sys_xtdrone.git
```
If you are in mainland China, we recommend you to update the submodules to point to gitee:

 revise .gitmodules
```
[submodule "mavlink/include/mavlink/v2.0"]
	path = mavlink/include/mavlink/v2.0
	url = https://gitee.com/robin_shaun/c_library_v2.git
	branch = master
[submodule "src/drivers/uavcan/libuavcan"]
	path = src/drivers/uavcan/libuavcan
	url = https://gitee.com/robin_shaun/uavcan.git
	branch = px4
[submodule "Tools/jMAVSim"]
	path = Tools/jMAVSim
	url = https://gitee.com/robin_shaun/jMAVSim.git
	branch = master
[submodule "Tools/sitl_gazebo"]
	path = Tools/sitl_gazebo
	url = https://gitee.com/jinxer000/sitl_gazebo_hitl.git
	branch = hitl_dev
[submodule "src/lib/matrix"]
	path = src/lib/matrix
	url = https://gitee.com/robin_shaun/Matrix.git
	branch = master
[submodule "src/lib/ecl"]
	path = src/lib/ecl
	url = https://gitee.com/robin_shaun/ecl.git
	branch = master
[submodule "boards/atlflight/cmake_hexagon"]
	path = boards/atlflight/cmake_hexagon
	url = https://gitee.com/robin_shaun/cmake_hexagon.git
	branch = px4
[submodule "src/drivers/gps/devices"]
	path = src/drivers/gps/devices
	url = https://gitee.com/robin_shaun/GpsDrivers.git
	branch = master
[submodule "src/modules/micrortps_bridge/micro-CDR"]
	path = src/modules/micrortps_bridge/micro-CDR
	url = https://gitee.com/robin_shaun/micro-CDR.git
	branch = px4
[submodule "platforms/nuttx/NuttX/nuttx"]
	path = platforms/nuttx/NuttX/nuttx
	url = https://gitee.com/robin_shaun/NuttX.git
	branch = px4_firmware_nuttx-9.1.0+
[submodule "platforms/nuttx/NuttX/apps"]
	path = platforms/nuttx/NuttX/apps
	url = https://gitee.com/robin_shaun/NuttX-apps.git
	branch = px4_firmware_nuttx-9.1.0+
[submodule "platforms/qurt/dspal"]
	path = platforms/qurt/dspal
	url = https://gitee.com/robin_shaun/dspal.git
[submodule "Tools/flightgear_bridge"]
	path = Tools/flightgear_bridge
	url = https://gitee.com/robin_shaun/PX4-FlightGear-Bridge.git
	branch = master 
[submodule "Tools/jsbsim_bridge"]
	path = Tools/jsbsim_bridge
	url = https://gitee.com/robin_shaun/px4-jsbsim-bridge.git
[submodule "src/examples/gyro_fft/CMSIS_5"]
	path = src/examples/gyro_fft/CMSIS_5
	url = https://gitee.com/mirrors/CMSIS_5
```
run 
```
git submodule update --init
cd ~/PX4_Firmware/src/drivers/uavcan/libuavcan
```
revise .gitmodules
```
[submodule "dsdl"]
	path = dsdl
	url = https://gitee.com/robin_shaun/dsdl
	branch = legacy-v0
[submodule "libuavcan/dsdl_compiler/pyuavcan"]
	path = libuavcan/dsdl_compiler/pyuavcan
	url = https://gitee.com/robin_shaun/pyuavcan
[submodule "libuavcan_drivers/kinetis"]
	path = libuavcan_drivers/kinetis
	url = https://gitee.com/robin_shaun/libuavcan_kinetis.git
```
```
git submodule update --init
cd ~/PX4_Firmware/Tools/jMAVSim
```
revise .gitmodules
```
[submodule "jMAVlib"]
	path = jMAVlib
	url = https://gitee.com/robin_shaun/jMAVlib
	branch = master
```
```
git submodule update --init
cd ~/PX4_Firmware/Tools/sitl_gazebo
```
```
git submodule update --init
cd ~/PX4_Firmware/Tools/sitl_gazebo
```

```
git submodule update --init --recursive
```
One additional step: 
```
  cp -r Tools/sitl_gazebo/models/*  ~/.gazebo/models/
  rm -r ~/.gazebo/models/stereo_camera/
  cp -r catkin_ws ~/
  cd ~/catkin_ws
  catkin_make
  ```

Also make sure cpc_aux_mapping, cpc_motion_planning, cpc_reference_publisher,cpc_ws in imav branch.
You may set up the environment of CPC by following the readme in cpc_ws.
Then compile two repos respectively.
For PX4_firmware, you can do
```
git tag -a v1.11.0 -m "init"
make px4_sitl_default gazebo
```
Finnaly, rename the root folder as 'PX4_firmware'.
# quick guide to launch
## Use simulator without offboard

```
cd PX4_Firmware
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
## indoor demo with offboard
```
cd PX4_Firmware
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 imav_indoor_rich.launch 

cd control_scripts
python multirotor_communication.py iris 0


cd cpc_ws
roslaunch cpc_aux_mapping hitl_sim.launch
roslaunch cpc_motion_planning motion_nf1_sitl.launch

roslaunch mobile_cylinder multi_box.launch
```
Then use 2d navigation goal to set target

## task-motion with BT
```
cd core_module/cpc_motion_planing
git checkout "emergency"


cd PX4_Firmware
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 imav_indoor_rich.launch 

cd control_scripts
python multirotor_communication.py iris 0


cd cpc_ws
roslaunch cpc_aux_mapping hitl_sim.launch
roslaunch cpc_motion_planning motion_nf1_sitl.launch

cd Behavior_tree_ws
roslaunch behavior_tree_leaves guard_robot_behavior_tree.launch
```
The task is go to (2,10,2).
Note the target cannot be set in RVIZ  for now.

## Dummy detection with YOLO
```
cd PX4_Firmware
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 imav_indoor_yolo.launch

cd control_scripts
python multirotor_communication_manual.py typhoon_h480 0
python gimbal_control.py typhoon_h480 0
python multirotor_keyboard_control.py typhoon_h480 1 vel

cd darknet_ros_ws
roslaunch darknet_ros task1.launch

```
or
```
python multirotor_communicationl.py typhoon_h480 0
roslaunch cpc_aux_mapping hitl_sim.launch  vehicle:=typhoon_h480_0
roslaunch cpc_motion_planning motion_nf1_sitl.launch vehicle:=typhoon_h480_0
```
## multi-agent demo with offboard
Since the target is same, agents will crash with each other.
A remedy is setting offsets in targets.
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
roslaunch px4 imav_indoor_hitl.launch 

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
you need to install gazebo_ros_link_attacher plugin following this:
```
mkdir -p gazebo_link_attacher_ws/src
cd gazebo_link_attacher_ws/src
catkin_init_workspace
git clone https://github.com/SwonGao/gazebo_ros_link_attacher.git
cd ..
catkin_make
```

Then, you 'd better copy the following lines into the ~/.bashrc file:
```
source ~/gazebo_link_attacher_ws/devel/setup.bash     #This should above the line "source px4..."
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_ros_link_attacher/devel/lib
```

```
cd PX4_Firmware
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_ros_link_attacher/devel/lib

roslaunch px4 amu_cooperative_assembly.launch
```


Then:
cd ~/XTDrone/communication
python multirotor_communication_manual.py iris 0
python multirotor_communication_manual.py iris 1
python multirotor_communication_manual.py iris 2

cd ~/XTDrone/control
python multirotor_keyboard_control.py iris 3 vel

After taking off, you need to delete the shelfs:
rosservice call /gazebo/delete_model "model_name: 'shelfs'"

## IMAV outdoor
```
cd PX4_Firmware
source ~/catkin_ws/devel/setup.bash   
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 imav_outdoor.launch 

cd apf_ws
roslaunch px4_controller px4_control.launch

cd cpc_ws
roslaunch cpc_aux_mapping hitl_sim.launch 
roslaunch cpc_motion_planning motion_nf1_sitl.launch

rosservice call /iris_0/engage
```
Then click on RVIZ to set targets.

If u want to try task:
```
cd NUC-CODE
roslaunch zbar_ros example.launch
cd ltl_ros_ws/
source devel/setup.bash
python src/P_MAS_TG/guo_thesis_algorithm/imav_outdoor_px4.py
```
## IMAV outdoor-2drone
```
cd PX4_Firmware
source ~/catkin_ws/devel/setup.bash   
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 imav_outdoor_2drone.launch 

cd apf_ws
roslaunch px4_controller px4_control_2drone.launch

cd cpc_ws
roslaunch cpc_aux_mapping hitl_sim.launch 
roslaunch cpc_motion_planning motion_nf1_sitl.launch

rosservice call /iris_0/engage
rosservice call /iris_1/engage
```
Then click on RVIZ to set targets.

NOTE: the output pose is $(vehicle_name)/ground_truth/state
If u want to try task:
```
cd NUC-CODE
roslaunch zbar_ros zbar_2drone.launch
cd ltl_ros_ws/
source devel/setup.bash
python src/P_MAS_TG/guo_thesis_algorithm/imav_outdoor_px4.py
```

## developing


roslaunch cpc_aux_mapping laser3d_sim.launch
roslaunch cpc_aux_mapping sim_uav.launch

## todo
the problem of outdoor 2 drone: the vision_pose cannot change local_pose. blocked px4_controller for now... use ref instead.  
we must past global position into px4, so that it can know the real target. if it always think that the initial pose is 0,0, then the tgt is wrong. 
1. odometry_out will cause tf issue:
2. /iris_0/mavros/vision_pose/pose cannot fuse with mavros/local_position/pose.

Since the mavros origin is always (0,0,0),
in multi demo, i set init_pose...
```
            self.initial_pose=Vector3(self.model_pose.x-self.local_pose.pose.position.x,
                                        self.model_pose.y-self.local_pose.pose.position.y,
                                        self.model_pose.z-self.local_pose.pose.position.z)
```
then in command_pose:
```
      cmd_local =Vector3(msg.pose.position.x-self.initial_pose.x,
                            msg.pose.position.y-self.initial_pose.y,
                            msg.pose.position.z-self.initial_pose.z) 
```

3. does vrpn node pub tf??
iris_1 ref_traj always 0???
## about texture (For Zhaiyu)
Scripts can be found in:
/usr/share/gazebo-9/media/materials/scripts$

But in this repo,  you can check Tools/sitl_gazebo/models/rover/material/   instead.


