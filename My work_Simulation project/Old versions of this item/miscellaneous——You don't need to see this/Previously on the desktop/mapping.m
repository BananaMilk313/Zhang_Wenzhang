function [subLiDAR,scan,velmsg,Odom,PubVel,OdomPos,SubOdom,rate,euler, ...
    startTime,yaw,yaw_list,x,y,vc_log,wc_log,va_log,wa_log,time_log, ...
    map_xy] ... 
    =mapping( ...
    map_xy,subLiDAR,scan,velmsg,Odom,PubVel,OdomPos,SubOdom,rate,euler, ...
    startTime,yaw,yaw_list,x,y,vc_log,wc_log,va_log,wa_log,time_log)



% === Step 1: 获取雷达数据 ===
scan = receive(subLiDAR);
dist = scan.ranges;

% === Step 2: 获取小车全局位姿 ===
currentPosition = [Odom.pose.pose.position.x, Odom.pose.pose.position.y];
currentOrientation = quat2eul([Odom.pose.pose.orientation.w, ...
    Odom.pose.pose.orientation.x, ...
    Odom.pose.pose.orientation.y, ...
    Odom.pose.pose.orientation.z]);
nowangle = currentOrientation(1); % 偏航角（yaw）


% 转换雷达数据到局部坐标系
dist_x = [];
dist_y = [];
for i = 1:360
    if isnan(dist(i))
        dist_x = [dist_x, 0];
        dist_y = [dist_y, 0];
    else
        angle = pi/180 * ((i-1) + 90); % 转换为弧度制，+90 是因为雷达扫描的角度偏移
        dist_x = [dist_x, dist(i) * sin(angle)];
        dist_y = [dist_y, dist(i) * cos(angle)];
    end
end

nowangle = -nowangle;

% === Step 3: 将局部坐标系点云转换到全局坐标系 ===
% 构造旋转矩阵
rotationMatrix = [cos(nowangle), -sin(nowangle);
                 sin(nowangle),  cos(nowangle)];
    
% 将雷达点从小车坐标系变换到全局坐标系
localPoints = [dist_x; dist_y]; % 局部点云坐标 (2 x N 矩阵)

globalPoints = [];
for i=1:360
    Points = rotationMatrix * localPoints(:,i); % 旋转到全局方向
    w=0;
    w = Points(1) + currentPosition(1);
    Points(1)=w;
    w = Points(2) - currentPosition(2);
    Points(2)=w;
    % globalPoints 是 2xN 的矩阵，第一行是 X 坐标，第二行是 Y 坐标
    globalPoints = [globalPoints Points];
end

map_xy=[map_xy(1, :) -globalPoints(2, :); map_xy(2, :) globalPoints(1, :);];



% === Step 5: 可视化全局地图 ===
figure(1);
plot(map_xy(1, :),map_xy(2, :), 'r.'); % 绘制全局地图点
axis equal;
set(gca, 'XDir', 'reverse'); % 显式设置X轴正方向朝zuo
title('Global Map');
xlabel('X (meters)');
ylabel('Y (meters)');
hold on;
plot(y,x,'c*-');%draw real path
hold off;
drawnow;
