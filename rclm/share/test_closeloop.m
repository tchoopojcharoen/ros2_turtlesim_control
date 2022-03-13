%turtlesim_node = TurtlesimNode;
controller_node = TurtlesimController;
scheduler_node = ros2node('scheduler');
scheduler_enable_cli = ros2svcclient(scheduler_node,"/enable","std_srvs/Empty");
scheduler_set_goal_cli = ros2svcclient(scheduler_node,"/set_goal","turtlesim_control_srv/SetGoal");
enable_req = ros2message(scheduler_enable_cli);
set_goal_req = ros2message(scheduler_set_goal_cli);

%%

call(scheduler_enable_cli,enable_req)

%%
set_goal_req.x = 4;
set_goal_req.y = 2;

call(scheduler_set_goal_cli,set_goal_req)

%%

delete(turtlesim_node)
delete(controller_node)
delete(scheduler_node)

pause(3)
ros2 node list