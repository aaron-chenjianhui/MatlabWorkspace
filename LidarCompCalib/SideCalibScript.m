clear;
file_name = './data/data_2.txt';
[angle_seq, lidar_data] = ReadData(file_name);

% % data 1
% LMIN = 65;
% LMAX = 115;
% RMIN = -115;
% RMAX = -60;
% dist = 6298;
% l_dist = 2817;
% r_dist = 3481;

% % data 2
% LMIN = 70;
% LMAX = 135;
% RMIN = -115;
% RMAX = -75;
% dist = 6298;
% l_dist = 1961;
% r_dist = 4336;

% % data 3
% LMIN = 51;
% LMAX = 120;
% RMIN = -100;
% RMAX = -60;
% dist = 6297;
% l_dist = 1868;
% r_dist = 4428;

% % data 4
% LMIN = 67;
% LMAX = 100;
% RMIN = -120;
% RMAX = -60;
% dist = 6297;
% l_dist = 3829;
% r_dist = 2466;

% data 5
LMIN = 80;
LMAX = 115;
RMIN = -125;
RMAX = -55;
dist = 6297;
l_dist = 4067;
r_dist = 2226;

%% Select lidar angle
% polar(angle_seq, lidar_data(:,1), 'g.');hold on;
%
% left_angle = deg2rad([LMIN, LMAX]);
% [angle_seq_left, lidar_data_left] = DataSelect(left_angle, angle_seq, lidar_data(:,1));
% polar(angle_seq_left, lidar_data_left, 'r.');
%
% right_angle = deg2rad([RMIN, RMAX]);
% [angle_seq_right, lidar_data_right] = DataSelect(right_angle, angle_seq, lidar_data(:,1));
% polar(angle_seq_right, lidar_data_right, 'r.');


%%
% pack_select = 3000;
num = size(lidar_data, 2);

comp_l = [];

% set RANSAC options
options.epsilon = 5;
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


for i = 1:1:num
    pack_select = i;
    
    %
    angle_seq_left = [];
    lidar_data_left = [];
    angle_seq_right = [];
    lidar_data_right = [];
    
    angle_seq_select = angle_seq;
    lidar_data_select = lidar_data(:,pack_select);
    
    % left angle
    left_angle = deg2rad([LMIN, LMAX]);
    [angle_seq_left, lidar_data_left] = DataSelect(left_angle, angle_seq_select, lidar_data_select);
    
    % right angle
    right_angle = deg2rad([RMIN, RMAX]);
    [angle_seq_right, lidar_data_right] = DataSelect(right_angle, angle_seq_select, lidar_data_select);
    
    right_x = lidar_data_right.*cos(angle_seq_right)*1000;
    right_y = lidar_data_right.*sin(angle_seq_right)*1000;
    left_x = lidar_data_left.*cos(angle_seq_left)*1000;
    left_y = lidar_data_left.*sin(angle_seq_left)*1000;
    
    right_ransac_x = linspace(min(right_x), max(right_x), 500);
    left_ransac_x = linspace(min(left_x), max(left_x), 500);
    
    right_data(1,:) = right_x;
    right_data(2,:) = right_y;
    left_data(1,:) = left_x;
    left_data(2,:) = left_y;
    
    
    
    % run RANSAC
    [right_result, right_opt] = RANSAC(right_data, options);
    [left_result, left_opt] = RANSAC(left_data, options);
    
    
    right_k = -right_result.Theta(1)/right_result.Theta(2);
    right_b = -right_result.Theta(3)/right_result.Theta(2);
    left_k = -left_result.Theta(1)/left_result.Theta(2);
    left_b = -left_result.Theta(3)/left_result.Theta(2);
    
    right_ransac_y = right_k*right_ransac_x + right_b;
    left_ransac_y = left_k*left_ransac_x + left_b;
    
    A_right = [-1/right_k -1; right_k -1];
    B_right = [0; -right_b];
    C_right = A_right \ B_right;
    
    A_left = [-1/left_k -1; left_k -1];
    B_left = [0; -left_b];
    C_left = A_left \ B_left;
    
%     plot(right_ransac_x, right_ransac_y, 'r-');hold on;axis equal;
%     plot([0 C_right(1)], [0 C_right(2)], 'b-');
%     plot(C_right(1), C_right(2), 'ro');
%     
%     plot(left_ransac_x, left_ransac_y);
%     plot([0 C_left(1)], [0 C_left(2)], 'b-');
%     plot(C_left(1), C_left(2), 'ro');
%     
%     hold off;
%     pause(0.08);clf;
    
    right_l = norm(C_right);
    left_l = norm(C_left);
    l = norm(C_right - C_left);
    r_comp = r_dist - right_l;
    l_comp = l_dist - left_l;
    comp = dist - l;
    comp_l = [comp_l comp];   
end
