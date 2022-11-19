### Simulation environment of BiQu (wip)
Simulation environment of Bimodal Quadruped MQP, used for testing control system implementation.

#### `ros2_control_solo` 
* Control Solo12 robot in Gazebo Simulation Environment
* Follows repository `ros2_control_demos`, `ros2_control_bolt`, and `gazebo_ros2_control`

#### Packages
* Check required packages at: `doc/Start.md`
* `gazebo_ros2_control` - stores `controllers.yaml`, launch
* `ros2_control_bringup`
* `ros2_description` - stores description files: urdf, xacro, mesh

#### Launching 
* `ros2 launch gazebo_ros2_control_bolt solo_system_position_only_gazebo.launch.py`

#### Controller
* Position
```
ros2 topic pub /joint_group_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0"
```

* or using publisher
```
ros2 run waypoints_controller position_publisher
```


#### Notes
11/18
* Solo12 Model: Correct Inertia, No Slipping
* Position JointGroupController, Effort JointTrajectoryController
<img src="https://i.postimg.cc/2jtcdKqW/solo-upright.png" width="500">
