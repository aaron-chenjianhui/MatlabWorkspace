function [ path_k, path_b, dest_x, dest_y, dest_theta, flag ] = LidarDataFcn( angle_seq, lidar_data )
%LidarDataFcn 此处显示有关此函数的摘要
%   此处显示详细说明


% 距前壁距离(mm)
front_dist = 1000;
front_thre = front_dist*1.5;
% 车体靠外
out_flag = 1;
flag = 1;

%% 数据预处理
% 排除不使用的点
filter_angle_range1 = [deg2rad(-270) deg2rad(-90)];
filter_data_range1 = [0.8 2];
filter_angle_range2 = [deg2rad(-90) deg2rad(90)];
filter_data_range2 = [0.8 10];
filter_angle_range3 = [deg2rad(90) deg2rad(270)];
filter_data_range3 = [0.8 2];
[angle_seq_select, lidar_data_select] = DataFilter(angle_seq, lidar_data, filter_angle_range1, filter_data_range1);
[angle_seq_select, lidar_data_select] = DataFilter(angle_seq_select, lidar_data_select, filter_angle_range2, filter_data_range2);
[angle_seq_select, lidar_data_select] = DataFilter(angle_seq_select, lidar_data_select, filter_angle_range3, filter_data_range3);

% 车的左侧
left_angle = deg2rad([0, 134]);
[angle_seq_left, lidar_data_left] = DataSelect(left_angle, angle_seq_select, lidar_data_select);
% 车的右侧
right_angle = deg2rad([-134, 0]);
[angle_seq_right, lidar_data_right] = DataSelect(right_angle, angle_seq_select, lidar_data_select);
% 车的前侧
front_angle = deg2rad([-90, 90]);
[angle_seq_front, lidar_data_front] = DataSelect(front_angle, angle_seq_select, lidar_data_select);

% 判断底盘是否过于进入集装箱
judge_angle = deg2rad([-45, 45]);
[angle_seq_judge, lidar_data_judge] = DataSelect(judge_angle, angle_seq_select, lidar_data_select);
% 单位mm
front_mean_dist = mean(lidar_data_judge)*1000;
if (front_mean_dist > front_thre)
    % 底盘比较靠近集装箱侧壁
    out_flag = 1;
else
    % 底盘比较靠近集装箱前壁
    out_flag = 0;
end

% % 转换为直角坐标
% total_x = lidar_data.*cos(angle_seq)*1000;
% total_y = lidar_data.*sin(angle_seq)*1000;
right_x = lidar_data_right.*cos(angle_seq_right)*1000;
right_y = lidar_data_right.*sin(angle_seq_right)*1000;
left_x = lidar_data_left.*cos(angle_seq_left)*1000;
left_y = lidar_data_left.*sin(angle_seq_left)*1000;

right_data(1,:) = right_x;
right_data(2,:) = right_y;
left_data(1,:) = left_x;
left_data(2,:) = left_y;


% set RANSAC options
options.epsilon = 1e-6;
options.P_inlier = 1.2;
options.sigma = 1;
options.est_fun = @estimate_line;
options.man_fun = @error_line;
options.mode = 'MSAC';
options.Ps = [];
options.notify_iters = [];
options.min_iters = 100;
options.fix_seed = false;
options.reestimate = true;
options.stabilize = false;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RANSAC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (out_flag == 1)
    % run RANSAC
    [right_result, right_opt] = RANSAC(right_data, options);
    [left_result, left_opt] = RANSAC(left_data, options);
    
    % 侧壁
    right_k = -right_result.Theta(1)/right_result.Theta(2);
    right_b = -right_result.Theta(3)/right_result.Theta(2);
    left_k = -left_result.Theta(1)/left_result.Theta(2);
    left_b = -left_result.Theta(3)/left_result.Theta(2);   
%     right_y = right_k*ransac_x + right_b;
%     left_y = left_k*ransac_x + left_b;
    
    % 前壁拟合
    front_angle_select(1) = max(angle_seq_right(right_result.CS));
    front_angle_select(2) = min(angle_seq_left(left_result.CS));
    [angle_seq_ign_front, lidar_data_ign_front] = DataSelect(front_angle_select, angle_seq_select, lidar_data_select);
    
    front_ign_data(1,:) = lidar_data_ign_front.*cos(angle_seq_ign_front)*1000;
    front_ign_data(2,:) = lidar_data_ign_front.*sin(angle_seq_ign_front)*1000;
    
    [front_result, front_opt] = RANSAC(front_ign_data, options);
    
    front_k = -front_result.Theta(1)/front_result.Theta(2);
    front_b = -front_result.Theta(3)/front_result.Theta(2);
%     front_y = front_k*ransac_x + front_b;
else
    front_x = lidar_data_front.*cos(angle_seq_front)*1000;
    front_y = lidar_data_front.*sin(angle_seq_front)*1000;
    front_data(1,:) = front_x;
    front_data(2,:) = front_y;
    
    front_options = options;
    front_options.P_inlier = 1.008;
    
    [front_result, front_opt] = RANSAC(front_data, front_options);
    
    % 前壁
    front_k = -front_result.Theta(1)/front_result.Theta(2);
    front_b = -front_result.Theta(3)/front_result.Theta(2);
%     front_y = front_k*ransac_x + front_b;
    
    [angle_seq_ign, lidar_data_ign] = DataIgn(angle_seq_front(front_result.CS), angle_seq_select, lidar_data_select);   
    [angle_seq_ign_left, lidar_data_ign_left] = DataSelect(left_angle, angle_seq_ign, lidar_data_ign);
    [angle_seq_ign_right, lidar_data_ign_right] = DataSelect(right_angle, angle_seq_ign, lidar_data_ign);
    
    left_ign_x = lidar_data_ign_left.*cos(angle_seq_ign_left)*1000;
    left_ign_y = lidar_data_ign_left.*sin(angle_seq_ign_left)*1000;
    right_ign_x = lidar_data_ign_right.*cos(angle_seq_ign_right)*1000;
    right_ign_y = lidar_data_ign_right.*sin(angle_seq_ign_right)*1000;
    
    left_ign_data(1,:) = left_ign_x;
    left_ign_data(2,:) = left_ign_y;
    right_ign_data(1,:) = right_ign_x;
    right_ign_data(2,:) = right_ign_y;
    
    % run RANSAC
    [right_result, right_opt] = RANSAC(right_ign_data, options);
    [left_result, left_opt] = RANSAC(left_ign_data, options);
    
    % 侧壁
    right_k = -right_result.Theta(1)/right_result.Theta(2);
    right_b = -right_result.Theta(3)/right_result.Theta(2);
    left_k = -left_result.Theta(1)/left_result.Theta(2);
    left_b = -left_result.Theta(3)/left_result.Theta(2);   
%     right_y = right_k*ransac_x + right_b;
%     left_y = left_k*ransac_x + left_b;
end

% 路径
path_k = (right_k + left_k)/2;
path_b = (right_b + left_b)/2;
% path_y = path_k*ransac_x + path_b;

% 停车位求解
stop_k = front_k;
if (stop_k<0)
    stop_b = front_b - front_dist*sqrt(front_k^2+1);
else
    stop_b = front_b + front_dist*sqrt(front_k^2+1);
end
% stop_y = stop_k*ransac_x + stop_b;

% 停车点以及姿态求解
% 位置
A = [stop_k -1; path_k -1];
B = [-stop_b; -path_b];
dest_pos = A \ B;
dest_x = dest_pos(1);
dest_y = dest_pos(2);
dest_theta = rad2deg(atan(path_k));

end

