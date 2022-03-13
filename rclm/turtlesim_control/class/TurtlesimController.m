classdef TurtlesimController < rclm_node
    %TURTLESIMCONTROLLER This ...
    % Object Properties and Methods.
    %
    % Node properties.
    %   Publisher_cmd_vel               - Publisher for /cmd_vel
    %   Subscriber_pose                 - Subscriber for /pose
    %   Service_set_goal                - Service server for /set_goal
    %   Service_enable_control          - Service server for /enable_control
    %   Client_notify_arrival           - Service client for /notify_arrival
    %
    % TurtlesimController methods:
    % TurtlesimController object construction:
    %   @TurtlesimController/TurtlesimController    - Construct node object.
    %
    %   delete                          - Delete and close node
    %

    % Copyright 2022 Pi Thanacha Choopojcharoen (GPL 2.0)
    
    properties (SetAccess=private)
        Goal = [2;3];
    end
    properties (Access=private)
        Pose
        Cmd_vel
        Mode
    end
    properties (Access=private,Constant)
        period = 0.1
        empty_req = ros2message("std_srvs/EmptyRequest");
    end

    methods
        function obj = TurtlesimController()
            %TURTLESIMCONTROLLER Construct TurtlesimController object.
            %
            %    N = TURTLESIMCONTROLLER() constructs and run a Turtlesim
            %    controller
            %
            %    Example:
            %       test_node = TurtlesimController();
            %
            %    See also DELETE
            obj@rclm_node('turtlesim_controller');
            obj.create_publisher("geometry_msgs/Twist","cmd_vel",10);
            sub = obj.create_subscription("turltlesim/Pose","pose",@obj.pose_callback,10);
            obj.Pose = ros2message(sub);
            obj.create_timer(obj.period,@obj.timer_callback);
            obj.create_service("turtlesim_control/SetGoal","/set_goal",@obj.set_goal_callback);
            obj.create_service("std_srvs/Empty","/enable",@obj.enable_callback);
            obj.create_client("std_srvs/Empty","/notify_arrival");

            obj.start_timer();
            
        end
        function delete(obj)
            %DELETE deconstructs this TurtlesimNode
            %   DELETE(OBJ) deconstruct the TurtlesimController.
            %
            %   Example:
            %       test_node = TurtlesimNController();
            %       delete(test_node)
            %       isempty(timerfind)
            %
            %    See also TURTLESIMCONTROLLER
            %
            
            delete@rclm_node(obj);
        end
    end
    methods (Access=private)        
        function pose_callback(obj,msg)
            obj.Pose = msg;
        end
        function timer_callback(obj,~,~)
            if obj.Mode == 1
                dp = obj.Goal-[obj.Pose.x;obj.Pose.y];
                dist = norm(dp);
                msg = ros2message(obj.Publishers{1});
                if dist > 0.2
                    [v,w] = obj.control(obj.Goal);
                    msg.linear.x = double(v);
                    msg.angular.z = double(w);
                    send(obj.Publishers{1},msg);
                else
                    msg.linear.x = double(0);
                    msg.angular.z = double(0);
                    obj.Mode = 0;
                    send(obj.Publishers{1},msg);
                    try
                        call(obj.Service_clients{1},obj.empty_req,"Timeout",3);
                    catch
                        disp('The server is busy, or no server to notify.')
                    end
                    disp('Robot is stopped.');
                end
            end
        end
        function [v,w] = control(obj,goal)
            dp = goal-[obj.Pose.x;obj.Pose.y];
            v = 1;
            e = atan2(dp(2),dp(1))-obj.Pose.theta;
            w = 10*atan2(sin(e),cos(e));
        end
        
        function resp = set_goal_callback(obj,req,resp)
            obj.Goal = [req.x;req.y];
            fprintf('Set Goal to : [%f,%f]\n',req.x,req.y);
            resp = ros2message("turtlesim_control/SetGoalResponse");
        end
        function resp = enable_callback(obj,req,resp)
            obj.Mode = 1;
            disp('Controller is enabled.');
        end
    end
end