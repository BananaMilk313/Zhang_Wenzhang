
%Please open the navigation & scan_wall & wallfollow functions simultaneously

% Clean up!
clear;
clear variables; close all; clc;

% Set ROS2 environment variables for the TurtleBot3 in Ubuntu
setenv('ROS_DOMAIN_ID', '30');

% Initialize ROS2 node
detectCmdVel = ros2node("/detectCmdVel");
% Setup access to LiDAR /scan topic
detectLiDAR = ros2node("/detectLiDAR");

% Create a Publisher object for the velocity command
PubVel = ros2publisher(detectCmdVel,'/cmd_vel','geometry_msgs/Twist');
% Create a subscriber for the /scan topic
subLiDAR = ros2subscriber(detectLiDAR, "/scan", "sensor_msgs/LaserScan");

% Create Subscribers for 'Odometry' and 'Model State' data
SubOdom = ros2subscriber(detectCmdVel, '/odom', 'nav_msgs/Odometry');
SubModelState = ros2subscriber(detectCmdVel, '/gazebo/model_states', 'gazebo_msgs/ModelStates');

% Create an object for the velocities
velmsg = ros2message(PubVel);

% Get odom data
Odom = receive(SubOdom);
% Get model state data
%ModelState = receive(SubModelState);

% Deconstruct odom data to obtain initial position
[OdomPos(1), OdomPos(2)] = deal(Odom.pose.pose.position.x, ...
    Odom.pose.pose.position.y);
% Deconstruct Model State date to obtain initial position

% Initialize the plot variables here
% Pre-allocation with appropriate size will increase efficiency; later!
x = OdomPos(1);
y = OdomPos(2);

% Obtain quaternions and convert them to Euler angles
euler = quat2eul([Odom.pose.pose.orientation.w, ...
    Odom.pose.pose.orientation.x, Odom.pose.pose.orientation.y, ...
    Odom.pose.pose.orientation.z]);
yaw = euler(1);  % Obtain yaw angle (Z-axis angle)
yaw_list = []; %Establish a yaw angle database

% Read most recent frame of LiDAR data
scan = receive(subLiDAR);
dist = scan.ranges; % copied LiDAR to 'dist' for convenience

rate = rateControl(10); % Set the loop rate to 10 Hz

time_log = [];
vc_log = [];  % log Commanded forward velocity
wc_log = [];  % log Commanded rotation rate
va_log = [];  % log Acquired forward velocity
wa_log = [];  % log Acquired rotation rate


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%% Set control parameters %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
maxLinearVelocity = 0.25;  % Maximum linear velocity (unit: m/s)
minLinearVelocity = 0.15; % Minimum linear velocity (avoid being too slow)
maxAngularVelocity = 1.00;   % Maximum angular velocity (in rad/s)
minAngularVelocity = 0.01; % Minimum angular velocity (avoid being too slow)
targetRadius = 0.2;       % Distance tolerance
angleTolerance = deg2rad(1);  % Heading tolerance (Converted to radians)


%{
%maxLinearVelocity = 0.4;  % Maximum linear velocity (unit: m/s)
%minLinearVelocity = 0.25; % Minimum linear velocity (avoid being too slow)

maxLinearVelocity = 0.1;  % Maximum linear velocity (unit: m/s)
minLinearVelocity = 0.01; % Minimum linear velocity (avoid being too slow)

%maxAngularVelocity = 1.45;   % Maximum angular velocity (in rad/s)
%minAngularVelocity = 0.01; % Minimum angular velocity (avoid being too slow)

maxAngularVelocity = 1;   % Maximum angular velocity (in rad/s)
minAngularVelocity = 0.01; % Minimum angular velocity (avoid being too slow)
targetRadius = 0.03;       % Distance tolerance
angleTolerance = deg2rad(1);  % Heading tolerance (Converted to radians)
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


currentPosition = [Odom.pose.pose.position.x, Odom.pose.pose.position.y];
% 输入起点和终点（需要手动设置）
start = [currentPosition(2), currentPosition(1)]; % 示例起点 (行, 列)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Drive the car to the starting point around the wall (-1, -1), %%%
% and then give the command to go straight. %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
map_xy=[inf;inf];
startTime = datetime('now');  % Get start time

%{
ISNAN=0;
ISback=0;
backtime=0;
tag=0;

% give the command to go straight
[subLiDAR,scan,velmsg,Odom,PubVel,OdomPos,SubOdom,rate,euler, ...
    startTime,yaw,yaw_list,x,y,vc_log,wc_log,va_log,wa_log,time_log, ...
    ISNAN,ISback,backtime,map_xy] ...
    =wallfollow(map_xy, ...
    0,1,ISNAN,ISback,backtime, ...
    maxLinearVelocity,minLinearVelocity,maxAngularVelocity,...
    minAngularVelocity,targetRadius,angleTolerance, ...
    subLiDAR,scan,velmsg,Odom,PubVel,OdomPos,SubOdom,rate,euler, ...
    startTime,yaw,yaw_list,x,y,vc_log,wc_log,va_log,wa_log,time_log);
if ISback==1
    tag=tag+1;
    ISback=0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


while ISNAN==0 && backtime==0
    angle_deg=0;

    % Measure the angle of the front wall
    [subLiDAR,scan,velmsg,Odom,PubVel,OdomPos,SubOdom,rate,euler, ...
        startTime,yaw,yaw_list,x,y,vc_log,wc_log,va_log,wa_log,time_log, ...
        angle_deg] ... %wall angle
        =scan_wall( ...
        subLiDAR,scan,velmsg,Odom,PubVel,OdomPos,SubOdom,rate,euler, ...
        startTime,yaw,yaw_list,x,y,vc_log,wc_log,va_log,wa_log,time_log);

    % Take the measured angle as the forward target
    [subLiDAR,scan,velmsg,Odom,PubVel,OdomPos,SubOdom,rate,euler, ...
        startTime,yaw,yaw_list,x,y,vc_log,wc_log,va_log,wa_log,time_log, ...
        ISNAN,ISback,backtime,map_xy] ...
        =wallfollow(map_xy, ...
        angle_deg,1,ISNAN,ISback,backtime, ...
        maxLinearVelocity,minLinearVelocity,maxAngularVelocity,...
        minAngularVelocity,targetRadius,angleTolerance, ...
        subLiDAR,scan,velmsg,Odom,PubVel,OdomPos,SubOdom,rate,euler, ...
        startTime,yaw,yaw_list,x,y,vc_log,wc_log,va_log,wa_log,time_log);
    
    if ISback==1 && tag ~=0
        backtime=1;
    end
end
%}

%Set target point set
goal=[1.5 -18];
numgoal = size(goal, 1);

start_t=start;
for i = 1:numgoal
    goal1=goal(i,1);
    goal2=goal(i,2);

    path_A = improved_A_Star(start_t, goal2, goal1);

    %path_A(:, 1) = linspace(start_t(1), goal2, 10)';
    %path_A(:, 2) = linspace(start_t(2), goal1, 10)';

    % === Step 5: 可视化全局地图 ===
    figure(1);
    hold on;
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % 绿色圆点表示起点
    plot(path_A(:,1), path_A(:,2), 'k*-');
    plot(goal2, goal1, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % 绿色圆点表示起点
    axis equal;
    set(gca, 'XDir', 'reverse'); % 显式设置X轴正方向朝zuo
    title('Global Map');
    xlabel('X (meters)');
    ylabel('Y (meters)');
    %hold off;
    drawnow;

    start_t=[goal2 goal1];
end


% Implement closed-loop control for each road sign
for i = 1:numgoal
    goal1=goal(i,1);
    goal2=goal(i,2);
    % Enable navigation function
    [subLiDAR,scan,velmsg,Odom,PubVel,OdomPos,SubOdom,rate,euler, ...
    startTime,yaw,yaw_list,x,y,vc_log,wc_log,va_log,wa_log,time_log,map_xy] ...
    =navigation( ...
    goal1,goal2, ...%End point
    start,map_xy,maxLinearVelocity,minLinearVelocity,maxAngularVelocity,...
    minAngularVelocity,targetRadius,angleTolerance, ...
    subLiDAR,scan,velmsg,Odom,PubVel,OdomPos,SubOdom,rate,euler, ...
    startTime,yaw,yaw_list,x,y,vc_log,wc_log,va_log,wa_log,time_log);
    start=[goal2 goal1];
    map_xy=[inf;inf];
end

figure(1);
plot(goal2, goal1, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % 绿色圆点表示起点

% Stop the robot
velmsg.linear.x = 0;
velmsg.angular.z = 0;
send(PubVel,velmsg);


%{
% === Step 5: 可视化全局地图 ===
figure(1);
plot(map_xy(1, :),map_xy(2, :), 'r.'); % 绘制全局地图点
axis equal;
set(gca, 'XDir', 'reverse'); % 显式设置X轴正方向朝zuo
title('Global Map');
xlabel('X (meters)');
ylabel('Y (meters)');
drawnow;
%}


% Calculate the drive time and output the result
total_time=seconds(datetime('now') - startTime);
fprintf('Drive Time (DT): %f sec\n\n',total_time);

% Calculate path distance covered and output the result
total_path_length = sum(sqrt(diff(x).^2 + diff(y).^2));
fprintf('Path Distance covered (PD): %f m\n\n',total_path_length);

% Calculate average commanded forward velocity and output the result
average_vc = mean(vc_log);
fprintf('Average Commanded Velocity (CV): %f (m/s)\n\n',average_vc);

% Calculate average commanded rotation rate and output the result
average_wc = mean(wc_log);
fprintf('Average Commanded Rotation Rate (CW): %f (rad/s)\n\n',average_wc);

% Calculate average acquired forward velocity and output the result
average_va = mean(va_log);
fprintf('Average Acquired Velocity (AV): %f (m/s)\n\n',average_va);

% Calculate average acquired rotation rate and output the result
average_wa = mean(wa_log);
fprintf('Average Acquired Rotation Rate (AW): %f (rad/s)\n\n',average_wa);

%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%% Drawing required plots %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the path
figure(1);
plot(x,y,'c*-')
hold on;
% Draw a circle for each target point with a radius of targetRadius, solid red line, unfilled
for j = size(goal, 1)
    viscircles(goal(j, :), targetRadius, 'EdgeColor', 'r', 'LineWidth', 1.5, 'LineStyle', '-');
    % Label coordinates next to each target point
    text(goal(j, 1), goal(j, 2), sprintf('(%.1f, %.1f)', goal(j, 1), goal(j, 2)), ...
        'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 10, 'Color', 'k');
end
% Equalize the axes
%axis equal
% Label the axes & add Title
xlabel('x')
ylabel('y')
title('Robot Path')
grid on
% Mark all the target points
hold off


% Aggregate all information into one plot for comparison
figure(7);
% Subplot 1: Variation of heading angle over time
subplot(3, 1, 1);
plot(time_log, yaw_list, 'r-', 'LineWidth', 1.5);   % 航向角 θ(t)
xlabel('Time (s)');
ylabel('Heading angle θ (rad)');
title('Heading angle θ(t)');
grid on;

% Subplot 2: Variation of commanded forward velocity and acquired forward velocity over time
subplot(3, 1, 2);
plot(time_log, vc_log, 'g--', 'LineWidth', 1.5);   % 指令线速度 v_c(t)
hold on;
plot(time_log, va_log, 'b-', 'LineWidth', 1.5);   % 实际线速度 v_a(t)
hold off;
xlabel('Time (s)');
ylabel('Forward velocity (m/s)');
legend('Commanded v_c(t)', 'Acquired v_a(t)');
title('Forward velocity v_c(t) & v_a(t)');
grid on;

% Subplot 3: Variation of commanded rotation rate and acquired rotation rate over time
subplot(3, 1, 3);
plot(time_log, wc_log, 'm--', 'LineWidth', 1.5);  % 指令角速度 ω_c(t)
hold on;
plot(time_log, wa_log, 'k-', 'LineWidth', 1.5);  % 实际角速度 ω_a(t)
hold off;
xlabel('Time (s)');
ylabel('Rotation rate (rad/s)');
legend('Commanded ω_c(t)', 'Acquired ω_a(t)');
title('Rotation rate ω_c(t) 和 ω_a(t)');
grid on;
%}