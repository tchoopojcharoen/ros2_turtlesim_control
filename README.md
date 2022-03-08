# ros2_turtlesim_control
This repository serves as a basic stepping stone for roboticists  who want to investigate fundamentals of ROS2 programming with Turtlesim. This repository also consists of basic implementation of mobile robot control using Python and Simulink (mutually exclusive).

For Python : \
Once you setup all packages and build your workspace, run the following command to execute launch file that will bring up Turtlesim and other nodes.
```
ros2 launch turtlesim_control turtlesim_bringup.launch.py 
```

In another terminal, you can run the following command to execute "go_to_goal" action. The turtle will automatically move to coordinate {x: 6.0, y: 1.0}

```
ros2 action send_goal --feedback /go_to_goal turtlesim_control_action/action/GoToGoal "{x: 6.0, y: 1.0}"
```

For Simulink:

You need to set up MATLAB's pyenv, cmake, and C++ compiler accordingly. And you also need "Mobile Robotics SImulation Toolbox" add-on. Once you have those, you can run the model without ROS2 network since the deafult model consists of its own simulator. 
-TO DO : instruction on how to run the model with Turtlesim in the network.
