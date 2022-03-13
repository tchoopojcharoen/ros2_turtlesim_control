classdef rclm_node < handle
%RCLM_NODE Object Properties and Methods.
%
% Node properties.
%   Name                    - Name of the node in ROS 2 network
%   Timer                   - Timer of the node
%   Publishers              - List of all publishers asscoiated with the node
%   Subscribers             - List of all subscribers asscoiated with the node
%   Service_servers         - List of all service servers asscoiated with the node
%   Service_clients         - List of all service clients asscoiated with the node
%
% rclm_node methods:
% rclm_node object construction:
%   @rclm_node/rclm_node    - Construct node object.
%
% General:
%   create_publisher        - Create a publisher with the node.
%   create_subscription     - Create a subscriber with the node.
%   create_timer            - Create a timer for the node.
%   create_service          - Create a service server of the node.
%   create_client           - Create a service client of the node.
%   delete                  - Delete the node from the memory.
%
% Execution:
%   start_timer             - Start timer of the node
%   stop_timer              - Stop timer of the node

% Copyright 2022 Pi Thanacha Choopojcharoen (GPL 2.0)
    properties (SetAccess=protected)
        Publishers = {}
        Subscribers = {}
        Service_servers = {}
        Service_clients = {}
        %Action_servers = 'to be implemented'
        %Action_clients = 'to be implemented'
    end
    properties (SetAccess=private)
        Name
        Timer
    end
    properties(Access=private)
        ThisNode
    end
    methods
        function obj = rclm_node(name)
        %RCLM_NODE Construct node object.
        %
        %    N = RCLM_NODE(NAME) constructs a ROS 2 node object in which 
        %    the given name NAME.
        %
        %    Example:
        %       % To construct a node object with a name /my_node:
        %         n = rclm_node('/my_node');
        %
        %    See also RCLM_NODE/CREATE_PUBLISHER, RCLM_NODE/CREATE_SUBSCRIPTION, RCLM_NODE/CREATE_TIMER, RCLM_NODE/CREATE_SERVICE, RCLM_NODE/CREATE_CLIENT.
            obj.Name = name;
            obj.ThisNode = ros2node(name);
        end

        function pub = create_publisher(obj,message_type,topic_name,qos_profile)
            %CREATE_PUBLISHER creates a publisher for the node
            %   PUB = CREATE_PUBLISHER(OBJ,TYPE,NAME,QOS) returns a 
            %   publisher PUB that publish a message of type TYPE to a topic 
            %   with the name NAME. The publisher has QOS depth of QOS.
            %
            %   Example:
            %       % Create a publisher for the node /talker 
            %       % which publish to a topic /cmd_vel and QOS depth of 10.
            %       % The message type is geometry_msgs/Twist
            %
            %       node_talker = rclm_node('/talker');
            %       pub = node_talker.create_publisher("geometry_msgs/Twist","/cmd_vel",10)
            %       node_talker.create_timer(0.5,@(obj,event)timer_callback(obj,event,pub));
            %       node_talker.start_timer();
            %       
            %       function timer_callback(obj, event, pub)
            %           msg = ros2message("geometry_msgs/Twist");
            %           msg.linear.x = randi(10);
            %           send(pub,msg);
            %       end
            %
            %   See also RCLM_NODE, CREATE_SUBSCRIPTION, CREATE_TIMER, CREATE_SERVICE, CREATE_CLIENT
            pub = ros2publisher(obj.ThisNode,topic_name,message_type,"Depth",qos_profile);
            obj.Publishers{end+1} = pub;
        end
        function sub = create_subscription(obj,message_type,topic_name,callback,qos_profile)
            %CREATE_SUBSCRIPTION creates a subscriber for the node
            %   SUB = CREATE_SUBSCRIPTION(OBJ,TYPE,NAME,CALLBACK,QOS) 
            %   returns a subscriber SUB that subscribe to a topic with the 
            %   name NAME with a callback CALLBACK. The message type is TYPE
            %   , and the subscriber has QOS depth of QOS.
            %
            %   Example:
            %       % Create a subscriber for the node /caller 
            %       % which subscribe to a topic /cmd_vel and QOS depth of 10.
            %       % The message type is geometry_msgs/Twist. Every time
            %       the message is sent, the subscriber will display its
            %       x-component.
            %
            %       node_talker = rclm_node('/talker');
            %       pub = node_talker.create_publisher("geometry_msgs/Twist","/cmd_vel",10);
            %       node_talker.create_timer(0.5,@(obj,event)timer_callback(obj,event,pub));
            %       node_caller = rclm_node('/caller');
            %       sub = node_caller.create_subscription("geometry_msgs/Twist","/cmd_vel",@sub_callback,10);
            %       node_talker.start_timer();
            % 
            %       function timer_callback(obj, event, pub)
            %           msg = ros2message("geometry_msgs/Twist");
            %           msg.linear.x = randi(10);
            %           send(pub,msg);
            %       end
            % 
            %       function sub_callback(msg)
            %           disp(msg.linear.x);
            %       end
            %
            %   See also RCLM_NODE, CREATE_PUBLISHER, CREATE_TIMER, CREATE_SERVICE, CREATE_CLIENT
            sub = ros2subscriber(obj.ThisNode,topic_name,message_type,callback,"Depth",qos_profile);
            obj.Subscribers{end+1} = sub;
        end
        function create_timer(obj,period,callback)
            %CREATE_TIMER creates a timer for the node
            %   CREATE_TIMER(OBJ,PERIOD,CALLBACK) attaches a timer that
            %   executea the callback CALLBACK every period PERIOD after 
            %   the node starts its timer.
            %
            %   Example:
            %       % Create a timer for a node /talker that 
            %       % publish a random number (1-10) as a "linear.x" 
            %       % component of a topic /cmd_vel every 0.5 seconds. 
            %       % The callback can be described as the following function.
            %
            %       node_talker = rclm_node('/talker');
            %       pub = node_talker.create_publisher("geometry_msgs/Twist","/cmd_vel",10);
            %       node_talker.create_timer(0.5,@(obj,event)timer_callback(obj,event,pub));
            %       nnode_talker.start_timer();
            %       
            %       function timer_callback(obj, event, pub)
            %           msg = ros2message("geometry_msgs/Twist");
            %           msg.linear.x = randi(10);
            %           send(pub,msg);
            %       end
            %
            %   See also RCLM_NODE, CREATE_PUBLISHER, CREATE_SUBSCRIPTION, CREATE_SERVICE, CREATE_CLIENT
            obj.Timer = timer("TimerFcn",callback,"Period",period,"ExecutionMode","fixedRate");
        end
        function srv = create_service(obj,serviec_type,service_name,callback)
            %CREATE_SERVICE creates a service server for a node
            %   SRV = CREATE_SERVICE(OBJ,TYPE,NAME,CALLBACK) returns a 
            %   service server SRV that provides a type of service TYPE 
            %   with the name NAME. The service behaves based on callback
            %   CALLBACK. 
            %
            %   Example:
            %       % Create a service server/say_hello that displays 
            %       % "hello" on a node /server. The service type is
            %       % std_srvs/Empty. 
            %
            %       node_server = rclm_node('/server');
            %       srv = create_service(node_server,"std_srvs/Empty","say_hello",@srv_callback)
            %
            %
            %       function resp = srv_callback(req,resp)
            %           disp("Hello")
            %       end
            %
            %   See also RCLM_NODE, CREATE_SUBSCRIPTION, CREATE_TIMER, CREATE_SERVICE, CREATE_CLIENT
            srv = ros2svcserver(obj.ThisNode,service_name,serviec_type,callback);
            obj.Service_servers{end+1} = srv;
        end
        function cli = create_client(obj,service_type,service_name)
            %CREATE_CLIENT creates a servie client for a node
            %   CLI = CREATE_CLIENT(OBJ,TYPE,NAME) returns a service
            %   client CLI that can call service server of name NAME, which
            %   provides a type of service TYPE.
            %
            %   Example:
            %       % Create a service client to a service /say_hello that 
            %       % displays "hello" on a another node and call the 
            %       % serviceThe service type is
            %       % std_srvs/Empty. 
            %
            %       node_server = rclm_node('/server');
            %       srv = create_service(node_server,"std_srvs/Empty","say_hello",@srv_callback)
            %       node_client = rclm_node('/caller');
            %       cli = create_client(node_client,"std_srvs/Empty","say_hello")
            %       req = ros2message(cli);
            %       waitForServer(cli,"Timeout",3);
            %       resp = call(cli,req,"Timeout",3);
            %       
            %       function resp = srv_callback(req,resp)
            %           disp("Hello")
            %       end
            %
            %   See also RCLM_NODE, CREATE_SUBSCRIPTION, CREATE_TIMER, CREATE_SERVICE, CREATE_CLIENT
            cli = ros2svcclient(obj.ThisNode,service_name,service_type);
            obj.Service_clients{end+1} = cli;
        end
        function delete(obj)
            %DELETE deconstructs this rclm_node
            %   DELETE(OBJ) deconstruct the rclm_node and its timer.
            %
            %   Example:            
            %       node = rclm_node('/random_pub');
            %       pub = node.create_publisher("geometry_msgs/Twist","/cmd_vel",10);
            %       node.create_timer(0.5,@(obj,event)timer_callback(obj,event,pub));
            %       node.start_timer()
            %       delete(node)
            %       isempty(timerfind)
            %       ~ishghandle(node)
            %
            %       function timer_callback(obj, event, pub)
            %           msg = ros2message("geometry_msgs/Twist");
            %           msg.linear.x = randi(10);
            %           send(pub,msg);
            %       end
            %

            if ~isempty(obj.Timer.TimerFcn)
               stop(obj.Timer);
               delete(obj.Timer);
            end
            delete(obj.ThisNode);
            delete@handle(obj);
            
        end 

        function start_timer(obj)
            %START_TIMER starts the timer of the node
            %   START_TIMER(OBJ) starts timer. However, if timer was not created,
            %   this will do nothing.
            %
            %   Example:
            %       % Create a timer for a node /talker that 
            %       % publish a random number (1-10) as a "linear.x" 
            %       % component of a topic /cmd_vel every 0.5 seconds. 
            %
            %       node_talker = rclm_node('/talker');
            %       pub = node_talker.create_publisher("geometry_msgs/Twist","/cmd_vel",10)
            %       node_talker.create_timer(0.5,@(obj,event)timer_callback(obj,event,pub));
            %       node_talker.start_timer();
            %       
            %       function timer_callback(obj, event, pub)
            %           msg = ros2message("geometry_msgs/Twist");
            %           msg.linear.x = randi(10);
            %           send(pub,msg);
            %       end
            %
            %   See also RCLM_NODE, CREATE_SUBSCRIPTION, CREATE_TIMER, CREATE_SERVICE, CREATE_CLIENT
            if ~isempty(obj.Timer.TimerFcn)
                start(obj.Timer);
            else
                warning('No timer is set.');
            end

        end
        function stop_timer(obj)
            %STOP_TIMER stop the timer of the node
            %   STOP_TIMER(OBJ) stop timer. However, if timer was not created,
            %   this will do nothing.
            %
            %   Example:
            %       % Create a timer for a node /talker that 
            %       % publish a random number (1-10) as a "linear.x" 
            %       % component of a topic /cmd_vel every 0.5 seconds. 
            %
            %       node_talker = rclm_node('/talker');
            %       pub = node_talker.create_publisher("geometry_msgs/Twist","/cmd_vel",10)
            %       node_talker.create_timer(0.5,@(obj,event)timer_callback(obj,event,pub));
            %       node_talker.start_timer();
            %       node_talker.stop_timer();
            %       
            %       function timer_callback(obj, event, pub)
            %           msg = ros2message("geometry_msgs/Twist");
            %           msg.linear.x = randi(10);
            %           send(pub,msg);
            %       end
            %
            %   See also RCLM_NODE, CREATE_SUBSCRIPTION, CREATE_TIMER, CREATE_SERVICE, CREATE_CLIENT
            if ~isempty(obj.Timer.TimerFcn)
                stop(obj.Timer);
            else
                warning('No timer is set.');
            end

        end   
    end
end