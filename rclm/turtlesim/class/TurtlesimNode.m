classdef TurtlesimNode < rclm_node
    %TURTLESIMNODE This is a stripped-down version of ROS Turtlesim
    % Object Properties and Methods.
    %
    % Node properties.
    %   Publisher_pose                  - Publisher for /pose
    %   Subscriber_cmd_vel              - Subscriber for /cmd_vel
    %   Service_clear                   - Service server for /clear
    %
    % TurtlesimNode methods:
    % TurtlesimNode object construction:
    %   @TurtlesimNode/TurtlesimNode    - Construct node object.
    %
    %   delete                          - Delete and close nodes and plots
    %

    % Copyright 2022 Pi Thanacha Choopojcharoen (GPL 2.0)
    properties (Access=private)
        Figure
        Axes
        Image
        AlphaChannel
        Plot
        CurrentSize
        PublisherCounter = 0
        PreviousPose
        Pose
        Message_cmd_vel
    end
    properties (Access=private,Constant)
        period = 0.1
        initial_pose = [5;5;0]
        size_window = [500,500]
        resolution = 50 % pixel/m
        x_min = 0
        x_max = 10
        y_min = 0
        y_max = 10
        width_cell = 2
        color_background = [69,86,255]/255
        color_grid = [150,150,255]/255
    end

    methods
        function obj = TurtlesimNode()
            %TURTLESIMNODE Construct TurtlesimNode object.
            %
            %    N = TURTLESIMNODE() constructs and run a Turtlesim with MATLAB
            %
            %    Example:
            %       test_node = TurtlesimNode();
            %
            %    See also DELETE
            obj@rclm_node('turtlesim_node');
            obj.create_publisher("turltlesim/Pose","pose",10);
            sub_cmd_vel = obj.create_subscription("geometry_msgs/Twist","cmd_vel",@obj.cmd_vel_callback,10);
            obj.Message_cmd_vel = ros2message(sub_cmd_vel);
            obj.create_timer(obj.period,@obj.timer_callback);
            obj.create_service("std_srvs/Empty","/clear",@obj.srv_callback);
            obj.Pose = obj.initial_pose;
            obj.set_figure();
            obj.set_axes();
            obj.load_turtle();
            obj.plot_turtle(obj.Pose);
            obj.start_timer();
        end
        function delete(obj)
            %DELETE deconstructs this TurtlesimNode
            %   DELETE(OBJ) deconstruct the TurtlesimNode and its plots.
            %
            %   Example:
            %       test_node = TurtlesimNode();
            %       delete(test_node)
            %       isempty(timerfind)
            %       ~ishghandle(test_node)
            %
            %    See also TURTLESIMNODE
            %
            if ishghandle(obj.Figure)
                delete(obj.Figure)
            end
            delete@rclm_node(obj);
        end
    end
    methods (Access=private)
        function set_figure(obj)
            obj.Figure = figure(1);
            set(obj.Figure,'name','Turtlesim','NumberTitle','off');
            position_fig = get(obj.Figure,'Position');
            position_fig(3) = obj.size_window(1)*6/5;
            position_fig(4) = obj.size_window(2)*6/5;
            set(obj.Figure,'Position',position_fig);
        end
        function set_axes(obj)
            obj.Axes = axes(obj.Figure,'Units','pixels');
            set(obj.Axes,'Position',[50,50,obj.size_window(1),obj.size_window(2)]);
            hold(obj.Axes,'on');
            obj.set_screen();
        end
        function set_screen(obj)
            size_background = [obj.x_min,obj.y_min,obj.x_max-obj.x_min,obj.y_max-obj.y_min];
            rectangle(obj.Axes,'Position',size_background*obj.resolution,'FaceColor',obj.color_background)
            axis(obj.Axes,'equal')
            axis(obj.Axes,[obj.x_min,obj.x_max,obj.y_min,obj.y_max]*obj.resolution)


            ticks_grid_x = ceil(obj.x_min):obj.width_cell:floor(obj.x_max);
            ticks_grid_y = ceil(obj.x_min):obj.width_cell:floor(obj.x_max);
            for x = ticks_grid_x
                line([x x]*obj.resolution,[obj.y_min obj.y_max]*obj.resolution,'LineStyle','--','color',obj.color_grid)
            end
            for y = ticks_grid_y
                line([obj.x_min obj.x_max]*obj.resolution,[y y]*obj.resolution,'LineStyle','--','color',obj.color_grid)
            end

            xticks(ticks_grid_x*obj.resolution)
            yticks(ticks_grid_y*obj.resolution)
            xticklabels(ticks_grid_x)
            yticklabels(ticks_grid_y)
        end
        function load_turtle(obj)
            path_file = mfilename('fullpath');
            file_in_turtlesim_package = 'class/turtlesim_node';
            path_package = path_file(1:end-length(file_in_turtlesim_package));
            path_images = [path_package '/image/'];

            file_images = {dir(path_images).name};
            file_images(1:2) = [];
            num_distro = numel(file_images);
            file_image = file_images{randi(num_distro)};

            [img, ~, alphachannel] = imread([path_images file_image]);
            obj.Image = img;
            obj.AlphaChannel = alphachannel;
        end
        function cmd_vel_callback(obj,msg)
            obj.Message_cmd_vel = msg;
        end
        function timer_callback(obj,~,~)
            if ~ishghandle(obj.Axes)
                try
                    stop(obj.Timer);
                    delete(obj);
                end
            else

                u = [obj.Message_cmd_vel.linear.x;obj.Message_cmd_vel.angular.z];
                obj.PreviousPose = obj.Pose;
                obj.Pose = obj.turtlesim_dynamics(obj.Pose,u);
                msg = ros2message("geometry_msgs/Pose2D");
                msg.x = single(obj.Pose(1));
                msg.y = single(obj.Pose(2));
                msg.theta = single(obj.Pose(3));
                msg.linear_velocity = single(obj.Message_cmd_vel.linear.x);
                msg.angular_velocity = single(obj.Message_cmd_vel.angular.z);
                send(obj.Publishers{1},msg);

                publish_rate = 0.1;
                if obj.PublisherCounter>=publish_rate/obj.period
                    obj.PublisherCounter = 0;
                    line(obj.Axes,[obj.PreviousPose(1) obj.Pose(1)]*obj.resolution,[obj.PreviousPose(2) obj.Pose(2)]*obj.resolution,'linewidth',3,'color',obj.color_grid)
                    delete(obj.Plot);
                    obj.plot_turtle(obj.Pose);
                    drawnow limitrate
                end
            end
            obj.PublisherCounter = obj.PublisherCounter + 1;

        end
        function plot_turtle(obj,pose)
            img = imrotate(obj.Image,-pose(3)*180/pi-90);
            alphachannel= imrotate(obj.AlphaChannel,-pose(3)*180/pi-90);
%             img = imresize(img,0.1);
%             alphachannel = imresize(alphachannel,0.1);
            obj.CurrentSize = size(img);
            obj.Plot = image(obj.Axes,pose(1)*obj.resolution-floor(obj.CurrentSize(1)/2),pose(2)*obj.resolution-floor(obj.CurrentSize(2)/2),img, 'AlphaData', alphachannel);
            drawnow('limitrate')
        end
        function x_new = turtlesim_dynamics(obj,x,u)
            dt = 0.1;
            f = @(x,u)[u(1)*cos(x(3));u(1)*sin(x(3));u(2)];
            k1 = f(x,u);
            k2 = f(x+k1*dt/2,u);
            k3 = f(x+k2*dt/2,u);
            k4 = f(x+k3*dt,u);
            dx = dt/6*(k1+2*k2+2*k3+k4);
            p_x = x(1);
            p_y = x(2);
            theta = x(3);
            if cos(theta)>0
                if p_x>=obj.x_max-0.75*obj.CurrentSize(1)/2/obj.resolution
                    p_x = obj.x_max-0.75*obj.CurrentSize(1)/2/obj.resolution;
                else
                    p_x = x(1)+dx(1);
                end
            elseif cos(theta)<0
                if p_x <=obj.x_min+0.75*obj.CurrentSize(1)/2/obj.resolution
                    p_x = obj.x_min+0.75*obj.CurrentSize(1)/2/obj.resolution;
                else
                    p_x = x(1)+dx(1);
                end
            else
                p_x = x(1);
            end
            if sin(theta)>0
                if p_y>=obj.y_max-0.75*obj.CurrentSize(2)/2/obj.resolution
                    p_y = obj.y_max-0.75*obj.CurrentSize(2)/2/obj.resolution;
                else
                    p_y = x(2)+dx(2);
                end
            elseif sin(theta)<0
                if p_y <=obj.y_min+0.75*obj.CurrentSize(2)/2/obj.resolution
                    p_y = obj.y_min+0.75*obj.CurrentSize(2)/2/obj.resolution;
                else
                    p_y = x(2)+dx(2);
                end
            else
                p_y = x(2);
            end

            theta = x(3)+dx(3);

            x_new = [p_x;p_y;theta];
        end
        function resp = srv_callback(obj,req,resp)
            obj.set_screen();
        end

    end
end