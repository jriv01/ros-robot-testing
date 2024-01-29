# ros-robot-testing
Repository for testing various ROS features &amp; robot implementations


## Packages
### rdt_legacy_robot
A package used for testing ROS & Gazebo simulations based on NYU RDT's 2023 **"AMIGO"** robot.

##### Launch files

| File name | Description |
|:-----------|:-------------|
|*rsp.launch.p<span>y* | A robot state publisher. Publishes the state of the links & joints as they are defined in the `description/robot.urdf.xacro` file. |
|*launch_sim.launch.py* | Invokes *rsp.launch.p<span>y*, launches a Gazebo world and spawns a model of the robot that is ready to be controlled. |

> When using the simulation, **`Twist`** messages published over the **`/cmd_vel`** topic are used to control the robot.
>
> The controller provided by the following command publishes such messages & can be used to control the robot. 
> `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
