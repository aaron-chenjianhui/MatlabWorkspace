clear;
file_name = './data/data_6.txt';
[angle_seq, lidar_data] = ReadData(file_name);


% data 6
FMIN = -35;
FMAX = 30;
dist = 2453;

% % data 7
% FMIN = -27;
% FMAX = 26;
% dist = 3462;

%% Select lidar angle
% polar(angle_seq, lidar_data(:,1), 'g.');hold on;
% 
% front_angle = deg2rad([FMIN, FMAX]);
% [angle_seq_front, lidar_data_front] = DataSelect(front_angle, angle_seq, lidar_data(:,1));
% polar(angle_seq_front, lidar_data_front, 'r.');




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
    angle_seq_front = [];
    lidar_data_front = [];

    
    angle_seq_select = angle_seq;
    lidar_data_select = lidar_data(:,pack_select);
    
    % front angle
    front_angle = deg2rad([FMIN, FMAX]);
    [angle_seq_front, lidar_data_front] = DataSelect(front_angle, angle_seq_select, lidar_data_select);
    

    

    front_x = lidar_data_front.*cos(angle_seq_front)*1000;
    front_y = lidar_data_front.*sin(angle_seq_front)*1000;
    

    front_ransac_x = linspace(min(front_x), max(front_x), 500);
    

    front_data(1,:) = front_x;
    front_data(2,:) = front_y;
    
    
    
    % run RANSAC

    [front_result, front_opt] = RANSAC(front_data, options);
    
    

    front_k = -front_result.Theta(1)/front_result.Theta(2);
    front_b = -front_result.Theta(3)/front_result.Theta(2);
    

    front_ransac_y = front_k*front_ransac_x + front_b;
    

    
    A_front = [-1/front_k -1; front_k -1];
    B_front = [0; -front_b];
    C_front = A_front \ B_front;
    
    
%     plot(front_ransac_x, front_ransac_y);
%     plot([0 C_front(1)], [0 C_front(2)], 'b-');
%     plot(C_front(1), C_front(2), 'ro');
%     hold off;
%     pause(0.08);clf;
    

    l = norm(C_front);
    comp = dist - l;
    comp_l = [comp_l comp];   
end