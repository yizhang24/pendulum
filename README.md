# Pendulum
Learning the basics of Gazebo and ROS 2 by stabilizing a simulated inverted pendulum.

## Structure
Contains two ROS 2 packages, `simulator` and `controller`.

The `simulator` packages contains SDF files describing the pendulum. The state of the system is published to the `joint_state` topic and the simulation listens for control input on the `force` topic. Run the simulation using:
```bash
ros2 launch simulator gazebo.launch.py
```

The `controller` package contains a node that subscribes to the topics set up by the simulation. A PID controller is the default:
```bash
ros2 run controller controller
```

Select the LQR controller with a ROS parameter:

```bash
ros2 run controller controller --ros-args -p controller_type:=lqr
```

The continuous-time LQR uses the state:

```
[cart position, cart velocity, pendulum angle, pendulum angular velocity]
```

The parameters are:

```
Q = diag(10, 1, 100, 10), R = 0.1
```

Which, after solving the CARE, produces the resulting gain ```K = [-10.0000, -16.4138, 145.1047, 62.4718]```

Both controllers use the same 50 N force limit.

## PID vs. LQR benchmark

Start the Docker container, build the workspace, and run:

```bash
docker exec ros2 bash -lc \
  '/root/project/scripts/benchmark_controllers.sh /root/project/benchmark_results'
```

The script runs both controllers from a fresh headless simulation. 
A separate `benchmarker` node applies the same 20 N·m pendulum torque 
for 0.1 simulated seconds, logs eight seconds of controller response 
to CSV, and prints angle, cart-position, settling-time, and control-effort metrics. 
Generated logs are placed in `benchmark_results/`.
