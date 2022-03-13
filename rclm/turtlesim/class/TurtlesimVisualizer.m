classdef TurtlesimVisualizer < matlab.System 
    % TURTLESIMVISUALIZER Turtlesim Visualizer for simulink only
    %
    % Displays the pose (position and orientation) of a turtlesim
    %
    %
    % Copyright 2022 Pi Thanacha Choopojcharoen (GPL 2.0)

    %% PROPERTIES
    % Public (user-visible) properties
    properties(Nontunable)
    end     
    properties(Nontunable, Logical)
    end
    properties
    end
    properties(Nontunable, Logical)
    end
    properties(Nontunable)
    end

    % Private properties
    properties(Access = private)
        fig;                % Figure window
        ax;                 % Axes for plotting
        Image;
        AlphaChannel;
        TurtleHandle;
        CurrentSize;
    end
    properties (Access=private,Constant)
        %period = 0.01
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

    %% METHODS
    methods(Access = protected)
        
        % Setup method: Initializes all necessary graphics objects
        function setupImpl(obj)
            % Create figure
            FigureName = 'Turtlesim';
            FigureTag = 'TurtlesimVisualization';
            existingFigures = findobj('type','figure','tag',FigureTag);
            if ~isempty(existingFigures)
                obj.fig = figure(existingFigures(1)); % bring figure to the front
                clf;
            else
                obj.fig = figure('Name',FigureName,'tag',FigureTag,'NumberTitle','off');
            end
            position_fig = get(obj.fig,'Position');
            position_fig(3) = obj.size_window(1)*6/5;
            position_fig(4) = obj.size_window(2)*6/5;
            set(obj.fig,'Position',position_fig);
            obj.set_axes();
            % Initialize robot plot
            obj.load_turtle;
            obj.plot_turtle(obj.initial_pose);
            
            
            % Final setup
            title(obj.ax,'Turtlesim Visualization');
            
            axis equal          
        end

        % Step method: Updates visualization based on inputs
        function stepImpl(obj,pose,varargin)          
            % Unpack the pose input into (x, y, theta)
            
            % Check for closed figure
            if ~isvalid(obj.fig)
                return;
            end
            % line(obj.Axes,[obj.PreviousPose(1) obj.Pose(1)]*obj.resolution,[obj.PreviousPose(2) obj.Pose(2)]*obj.resolution,'linewidth',3,'color',obj.color_grid) 
            % Update the robot pose
            delete(obj.TurtleHandle);
            obj.plot_turtle(pose);
            
            
            % Update the figure
            drawnow('limitrate')
            
        end

        % Define total number of inputs for system with optional inputs
        function num = getNumInputsImpl(obj)
            num = 1;
        end
        
        % Define input port names
        function [namePose,varargout] = getInputNamesImpl(obj)
            namePose = 'pose';
        end

        % Define icon for System block
        function icon = getIconImpl(~)
            icon = {'Turtlesim','Visualizer'};
        end
        
    end
    
    methods (Access = public)        
        
        function set_axes(obj)
            obj.ax = axes(obj.fig,'Units','pixels');
            set(obj.ax,'Position',[50,50,obj.size_window(1),obj.size_window(2)]);
            axis(obj.ax,'equal')
            hold(obj.ax,'on');
            obj.set_screen();
        end
        function set_screen(obj)
            size_background = [obj.x_min,obj.y_min,obj.x_max-obj.x_min,obj.y_max-obj.y_min];
            rectangle(obj.ax,'Position',size_background*obj.resolution,'FaceColor',obj.color_background)
            axis(obj.ax,'equal')
            axis(obj.ax,[obj.x_min,obj.x_max,obj.y_min,obj.y_max]*obj.resolution)


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
            file_in_turtlesim_package = 'class/TurtlesimVisualizer';
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
        function plot_turtle(obj,pose)
            img = imrotate(obj.Image,-pose(3)*180/pi-90);
            alphachannel= imrotate(obj.AlphaChannel,-pose(3)*180/pi-90);
            obj.CurrentSize = size(img);
            obj.TurtleHandle = image(obj.ax,pose(1)*obj.resolution-floor(obj.CurrentSize(1)/2),pose(2)*obj.resolution-floor(obj.CurrentSize(2)/2),img, 'AlphaData', alphachannel);
            axis(obj.ax,[obj.x_min,obj.x_max,obj.y_min,obj.y_max]*obj.resolution)

        end
    end
    
    methods (Static, Access = protected)
        % Do not show "Simulate using" option
        function flag = showSimulateUsingImpl
            flag = false;
        end
        % Always run in interpreted mode
        function simMode = getSimulateUsingImpl
            simMode = 'Interpreted execution';
        end
    end
        
end