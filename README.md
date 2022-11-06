### Simulation environment of BiQu (wip)
Simulation environment of Bimodal Quadruped MQP, used for testing control system implementation.

#### `ros2_control_solo` 
* Control Solo12 robot in Gazebo Simulation Environment
* Follows repository `ros2_control_demos`, `ros2_control_bolt`, and `gazebo_ros2_control`

#### Packages
* `gazebo_ros2_control` - stores `controllers.yaml`, launch
* `ros2_control` 
* `ros2_control_bringup`
* `ros2_description` - stores description files: urdf, xacro, mesh

#### Notes
11/5
* Successfully spawned Solo12 model in Gazebo
* Solo12 oscillates upon spawning
  * test with extra dummy base link
  * test with attaching Solo12 on a crane
* Experiment with different controllers interfaces
  * Currently using ForwardPositionController
  * test with position interface, then add effort
<img src="https://i.postimg.cc/SQcTcmjL/first-spawn.png" width="500">
