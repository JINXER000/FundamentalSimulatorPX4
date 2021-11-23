# Code Review

## Error & Warning that can be ignored

```bash
ERROR [param] Parameter COM_CPU_MAX not found.

[ERROR] [1632650806.653412, 4331.668000]: Spawn service failed. Exiting.

[vehicle_spawn_ray_PC_3805_5889391543400089409-5] process has died [pid 3883, exit code 1, cmd /opt/ros/melodic/lib/gazebo_ros/spawn_model -sdf -file /home/ray/PX4_Firmware/Tools/sitl_gazebo/models/iris_2d_lidar/iris_2d_lidar.sdf -model iris_0 -x 0 -y 0 -z 1 -R 0 -P 0 -Y 0 __name:=vehicle_spawn_ray_PC_3805_5889391543400089409 __log:=/home/ray/.ros/log/73c91f0e-1eb1-11ec-825c-14f6d87ec6ba/vehicle_spawn_ray_PC_3805_5889391543400089409-5.log].

Warning [parser.cc:950] XML Element[scale], child of element[model] not defined in SDF. Ignoring[scale]. You may have an incorrect SDF file, or an sdformat version that doesnt support this element.

In Gazebo Simulation, Stereo Camera is in the Air.
```

## Fundamental Code Trouble shooting (Version b212b0c)

1. For Version ``b212b0c`` and before, after cloning the code, please remove the ``PX4_Firmware/Tools/sitl_gazebo/models/ihumanv1.zip``

2. If using SITL, please copy the ``iris.sdf`` file of the original PX4_Firmware from ``PX4_Firmware/Tools/sitl_gazebo/models/iris/iris.sdf`` to the same folder in ``fundamental_sys_xtdrone``

3. For this version, stereo Lidar is not applicable, so, please modify the ``PX4_Firmware/launch/imav_indoor_rich.launch`` to enable the 2d Lidar. Specifically, comment Line 16 and uncomment Line 14.

## CPC Code Related Configuration

1. If you are using a lookup table controller, you could download some sample controllers from:
[here.](https://drive.google.com/open?id=1R_iP5SmkYX8zVNzoXnETMrqD30_yM2ms)   
The controller are loaded through the ``load_data()`` function in the   ``cpc/core_modules/cpc_motion_planning/include/cpc_motion_planning/uav/controller/uav_dp_control.h``.  
You might want to modify the path to load the controller files.

2. Have to comment ANY ``usrl_msgs`` related code in the ``/home/ray/catkin_ws/src/cpc/core_modules/cpc_reference_publisher/src/main.cpp``

3. Before running CPC related code, Please check the installation and PATH setting of CUDA using
```bash
nvcc --version
```
If the output looks like below, it means the related dependencies are completed.
```bash
nvcc: NVIDIA (R) Cuda compiler driver
Copyright (c) 2005-2019 NVIDIA Corporation
Built on Wed_Oct_23_19:24:38_PDT_2019
Cuda compilation tools, release 10.2, V10.2.89
```