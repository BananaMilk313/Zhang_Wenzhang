function [subLiDAR,scan,velmsg,Odom,PubVel,OdomPos,SubOdom,rate,euler, ...
    startTime,yaw,yaw_list,x,y,vc_log,wc_log,va_log,wa_log,time_log] ...
    =navigation( ...
    map_xy,end_x,end_y, ...%End point
    maxLinearVelocity,minLinearVelocity,maxAngularVelocity,...
    minAngularVelocity,targetRadius,angleTolerance, ...
    subLiDAR,scan,velmsg,Odom,PubVel,OdomPos,SubOdom,rate,euler, ...
    startTime,yaw,yaw_list,x,y,vc_log,wc_log,va_log,wa_log,time_log)

% Define the current target location
target = [end_x,end_y];
    
% Obtain current odometer information
currentPosition = [Odom.pose.pose.position.x, Odom.pose.pose.position.y];
currentOrientation = quat2eul([Odom.pose.pose.orientation.w, ...
    Odom.pose.pose.orientation.x, ...
    Odom.pose.pose.orientation.y, ...
    Odom.pose.pose.orientation.z]);
    
% Calculate the distance to the target point
distanceToTarget = norm(target - currentPosition);

% Read most recent frame of LiDAR data
scan = receive(subLiDAR);
dist = scan.ranges; % copied LiDAR to 'dist' for convenience

% Move to the main loop of the target point
while distanceToTarget > targetRadius
    % Calculate the angle to the target point
    targetAngle = atan2(target(2) - currentPosition(2), target(1) - currentPosition(1));
    angleDifference = wrapToPi(targetAngle - currentOrientation(1));
    
    % Read most recent frame of LiDAR data
    scan = receive(subLiDAR);
    dist = scan.ranges; % copied LiDAR to 'dist' for convenience
    [Min, IMin] = min(scan.ranges);
    IMin=IMin-1; % start index IMin at 0 rather than 1 degree
    Minangle = wrapToPi(pi/180*IMin); % Convert to the perspective required by the program

    if Min<0.7 % Start decelerating when the distance is less than 0.7
        linearSpeed = minLinearVelocity + (maxLinearVelocity*0.5 - minLinearVelocity) * exp(-5 * (0.6 - double(Min)) / 0.6);
        velmsg.linear.x = linearSpeed;
    else
        % Linear velocity control logic - smooth deceleration
        if distanceToTarget > 0.6
            linearSpeed = maxLinearVelocity;
        else
        % Use smooth exponential decay for deceleration instead of direct linear deceleration
            linearSpeed = minLinearVelocity + (maxLinearVelocity - minLinearVelocity) * exp(-1 * (0.5 - distanceToTarget) / 0.5);
        end
        velmsg.linear.x = linearSpeed;
    end
    

    if Min<0.5 % Start turning when the distance is less than 0.5
        angularSpeed = minAngularVelocity + (maxAngularVelocity - minAngularVelocity) * exp(-1 * double(Min));
        velmsg.angular.z = angularSpeed * -sign(Minangle);
    else
        % Rotation rate control logic - conservative steering to reduce overshoot
        if abs(angleDifference) > deg2rad(30)  % When the angle error is greater than 30 degrees, use a higher rotation rate
            angularSpeed = maxAngularVelocity;  % Reduce the maximum rotation rate
        else
        % Use smooth rotation rate attenuation control to gradually reduce the rotation rate according to the angle error
            angularSpeed = minAngularVelocity + (maxAngularVelocity - minAngularVelocity) * (abs(angleDifference) / deg2rad(30));
        end
        velmsg.angular.z = angularSpeed * sign(angleDifference);
    end

    % Secondary correction of rotation rate
    if Min>0.3 && abs(angleDifference) > deg2rad(75) % When there is a certain distance from the obstacle and the yaw angle is too large
        % let the non obstacle avoidance mode take over the rotation rate.
        if abs(angleDifference) > deg2rad(30)
            angularSpeed = maxAngularVelocity;
        else
            angularSpeed = minAngularVelocity + (maxAngularVelocity - minAngularVelocity) * (abs(angleDifference) / deg2rad(30));
        end
        velmsg.angular.z = angularSpeed * sign(angleDifference);
    end

    % Send speed command
    send(PubVel, velmsg);
        
    % Update current odometer information
    Odom = receive(SubOdom, 10);
    currentPosition = [Odom.pose.pose.position.x, Odom.pose.pose.position.y];
    currentOrientation = quat2eul([Odom.pose.pose.orientation.w, ...
        Odom.pose.pose.orientation.x, ...
        Odom.pose.pose.orientation.y, ...
        Odom.pose.pose.orientation.z]);
    
    % Calculate the distance to the target point
    distanceToTarget = norm(target - currentPosition);
    
    % Augment plot variables here with new data point
    x = [x currentPosition(1)];
    y = [y currentPosition(2)];
    
    yaw = currentOrientation(1);  % Obtain yaw angle (Z-axis angle)
    yaw_list = [yaw yaw_list]; %Establish a yaw angle database

    % log time
    currentTime = datetime('now') - startTime;
    % Calculate the difference between the current time and the start time
    time_log(end+1) = seconds(currentTime);

    % Commanded v&w
    vc = velmsg.linear.x;
    wc = velmsg.angular.z;

    % log Commanded v&w
    vc_log(end+1) = vc;
    wc_log(end+1) = wc;

    % Acquired v&w
    va = sqrt(Odom.twist.twist.linear.x^2 + Odom.twist.twist.linear.y^2);
    wa = Odom.twist.twist.angular.z;

    % log Acquired v&w
    va_log(end+1) = va;
    wa_log(end+1) = wa;
    
    % Wait to maintain the loop rate
    waitfor(rate);

    % Augment plot variables here with new data point
    %x_path = [x_path currentPosition(1)];
    %y_path = [y_path -currentPosition(2)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % 将路径叠加到地图上显示
    figure(2);
    hold on;    
    plot(y,x,'r*-');%draw real path
    hold off;
    drawnow;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


[subLiDAR,scan,velmsg,Odom,PubVel,OdomPos,SubOdom,rate,euler, ...
    startTime,yaw,yaw_list,x,y,vc_log,wc_log,va_log,wa_log,time_log, ...
    map_xy] ... 
    =mapping( ...
    map_xy,subLiDAR,scan,velmsg,Odom,PubVel,OdomPos,SubOdom,rate,euler, ...
    startTime,yaw,yaw_list,x,y,vc_log,wc_log,va_log,wa_log,time_log);

end
