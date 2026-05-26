# Pendulum
Learning the basics of Gazebo and ROS2 by stabilizing a simulated inverted pendulum.

## Structure
Contains two ROS2 packages, `simulator` and `controller`.

The `simulator` packages contains SDF files describing the pendulum. The state of the system is published to the `joint_state` topic and the simulation listens for control input on the `force` topic. Run the simulation by running:
```bash
ros2 launch simulator gazebo.launch.py
```

The `controller` package contains a node that subscribes to the topics set up by the simulation and uses 2 PID controllers to balance the pendulum and center the cart. Run the controller using:
```bash
ros2 run controller controller
```
## Demo
https://github.com/user-attachments/assets/d0b9a5af-0679-4554-a2bf-c50bd1d5e598
