# ros2_turtlesim_control
This repository serves as a basic stepping stone for roboticists  who want to investigate fundamentals of ROS2 programming with Turtlesim.

Once you setup all packages and build your workspace, run the following command to execute launch file that will bring up Turtlesim and other nodes.
```
ros2 launch turtlesim_control turtlesim_bringup.launch.py 
```

In another terminal, you can run the following command to execute "go_to_goal" action. The turtle will automatically move to coordinate {x: 6.0, y: 1.0}

```
ros2 action send_goal --feedback /go_to_goal turtlesim_control_action/action/GoToGoal "{x: 6.0, y: 1.0}"
```