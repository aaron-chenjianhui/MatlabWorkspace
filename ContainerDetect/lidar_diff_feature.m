% clear;
file_name = './Data/lidar_data.txt';
% [angle_seq, lidar_data, intensity_data] = ReadData(file_name);

% ��ǰ�ھ���(mm)
front_dist = 1000;
front_thre = front_dist*1.5;
% ���忿��
out_flag = 1;

% ȡ����һ���ֽ��д���
% pack_select = 3000;
num = size(lidar_data, 2);


pack_select = 1;

% ��ʼ��
angle_seq_left = [];
lidar_data_left = [];
angle_seq_right = [];
lidar_data_right = [];
front_data = [];
right_data = [];
left_data = [];

angle_seq_select = [];
lidar_data_select = [];
intensity_data_select = [];
[angle_seq_select, lidar_data_select, intensity_data_select] = ...
    DataFilter(angle_seq, lidar_data(:,pack_select), intensity_data(:,pack_select));

polar(angle_seq_select, lidar_data_select, 'b.');
%     polar(angle_seq, lidar_data(:,pack_select), 'b.');

data_diff = diff(lidar_data_select*1000);
data_diff_diff = diff(data_diff);
figure;plot(angle_seq_select*180/3.14, lidar_data_select, 'b.');
% plot(intensity_data_select, 'r.');
% plot(data_diff*1000, 'g');




% �������
% left_angle = deg2rad([30, 120]);
left_angle = deg2rad([0, 134]);
[angle_seq_left, lidar_data_left] = DataSelect(left_angle, angle_seq_select, lidar_data_select);
% �����Ҳ�
% right_angle = deg2rad([-120, -30]);
right_angle = deg2rad([-134, 0]);
[angle_seq_right, lidar_data_right] = DataSelect(right_angle, angle_seq_select, lidar_data_select);
% ����ǰ��
front_angle = deg2rad([-90, 90]);
[angle_seq_front, lidar_data_front] = DataSelect(front_angle, angle_seq_select, lidar_data_select);

% �жϵ����Ƿ���ڽ��뼯װ��
judge_angle = deg2rad([-45, 45]);
[angle_seq_judge, lidar_data_judge] = DataSelect(judge_angle, angle_seq_select, lidar_data_select);
% ��λmm
front_mean_dist = mean(lidar_data_judge)*1000;
if (front_mean_dist > front_thre)
    out_flag = 1;
else
    out_flag = 0;
end

% % angle_seq_handle = angle_seq(330:510);
% % lidar_data_handle = lidar_data(330:510,1000);
% angle_seq_handle = angle_seq;
% lidar_data_handle = lidar_data(:,1000);

% ת��Ϊֱ�����
total_x = lidar_data(:,pack_select).*cos(angle_seq)*1000;
total_y = lidar_data(:,pack_select).*sin(angle_seq)*1000;
right_x = lidar_data_right.*cos(angle_seq_right)*1000;
right_y = lidar_data_right.*sin(angle_seq_right)*1000;
left_x = lidar_data_left.*cos(angle_seq_left)*1000;
left_y = lidar_data_left.*sin(angle_seq_left)*1000;

%     hold on;
%     plot(front_x, front_y, 'b.');

ransac_x = linspace(min(total_x), max(total_x), 500);

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
    front_ign_data = [];
    
    % run RANSAC
    [right_result, right_opt] = RANSAC(right_data, options);
    [left_result, left_opt] = RANSAC(left_data, options);
    
    % ���
    right_k = -right_result.Theta(1)/right_result.Theta(2);
    right_b = -right_result.Theta(3)/right_result.Theta(2);
    left_k = -left_result.Theta(1)/left_result.Theta(2);
    left_b = -left_result.Theta(3)/left_result.Theta(2);
    
    right_y = right_k*ransac_x + right_b;
    left_y = left_k*ransac_x + left_b;
    
    % ǰ�����
    front_angle_select(1) = max(angle_seq_right(right_result.CS));
    front_angle_select(2) = min(angle_seq_left(left_result.CS));
    %         [angle_seq_ign, lidar_data_ign] = DataIgn([angle_seq_right(right_result.CS);angle_seq_left(left_result.CS)], angle_seq_select, lidar_data_select);
    [angle_seq_ign_front, lidar_data_ign_front] = DataSelect(front_angle_select, angle_seq_select, lidar_data_select);
    
    front_ign_data(1,:) = lidar_data_ign_front.*cos(angle_seq_ign_front)*1000;
    front_ign_data(2,:) = lidar_data_ign_front.*sin(angle_seq_ign_front)*1000;
    plot(front_ign_data(1,:), front_ign_data(2,:), 'b.');
    
    
    
    %         front_data(1,:) = [right_data(1, ~right_result.CS) left_data(1, ~left_result.CS)];
    %         front_data(2,:) = [right_data(2, ~right_result.CS) left_data(2, ~left_result.CS)];
    [front_result, front_opt] = RANSAC(front_ign_data, options);
    
    front_k = -front_result.Theta(1)/front_result.Theta(2);
    front_b = -front_result.Theta(3)/front_result.Theta(2);
    front_y = front_k*ransac_x + front_b;
    % hold on;
    %
    %         plot(front_data(1,:), front_data(2,:), 'r.');
    % plot(ransac_x, front_y, 'cyan-', 'LineWidth', 1);
else
    left_ign_data = [];
    right_ign_data = [];
    
    front_x = lidar_data_front.*cos(angle_seq_front)*1000;
    front_y = lidar_data_front.*sin(angle_seq_front)*1000;
    front_data(1,:) = front_x;
    front_data(2,:) = front_y;
    
    front_options = options;
    front_options.P_inlier = 1.008;
    
    [front_result, front_opt] = RANSAC(front_data, front_options);
    
    % ǰ��
    front_k = -front_result.Theta(1)/front_result.Theta(2);
    front_b = -front_result.Theta(3)/front_result.Theta(2);
    
    front_y = front_k*ransac_x + front_b;
    %         plot(ransac_x, front_y, 'black-', 'LineWidth', 1);
    
    [angle_seq_ign, lidar_data_ign] = DataIgn(angle_seq_front(front_result.CS), angle_seq_select, lidar_data_select);
    %         polar(angle_seq_ign, lidar_data_ign);
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
    
    %         plot( front_data(1,front_result.CS), front_data(2,front_result.CS), 'g.');
    %         plot(left_ign_x, left_ign_y, 'r.');hold on;
    %         plot(right_ign_x, right_ign_y, 'b.');
    
    % run RANSAC
    [right_result, right_opt] = RANSAC(right_ign_data, options);
    [left_result, left_opt] = RANSAC(left_ign_data, options);
    
    % ���
    right_k = -right_result.Theta(1)/right_result.Theta(2);
    right_b = -right_result.Theta(3)/right_result.Theta(2);
    left_k = -left_result.Theta(1)/left_result.Theta(2);
    left_b = -left_result.Theta(3)/left_result.Theta(2);
    
    right_y = right_k*ransac_x + right_b;
    left_y = left_k*ransac_x + left_b;
end

% ·��
path_k = (right_k + left_k)/2;
path_b = (right_b + left_b)/2;
path_y = path_k*ransac_x + path_b;

% ͣ��λ���
stop_k = front_k;
if (stop_k<0)
    stop_b = front_b - front_dist*sqrt(front_k^2+1);
else
    stop_b = front_b + front_dist*sqrt(front_k^2+1);
end
stop_y = stop_k*ransac_x + stop_b;

% ͣ�����Լ���̬���
% λ��
A = [stop_k -1; path_k -1];
B = [-stop_b; -path_b];
dest_pos = A \ B;
dest_x = dest_pos(1);
dest_y = dest_pos(2);
dest_theta = rad2deg(atan(path_k));


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Results Visualization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     figure;
hold on
% Raw Data
h_raw = plot(total_x, total_y, 'b.', 'MarkerSize', 5);
% ���
h_right = plot(ransac_x, right_y, 'g-', 'LineWidth', 1);
plot(ransac_x, left_y, 'g-', 'LineWidth', 1);
% ·��
h_path = plot(ransac_x, path_y, 'cyan--', 'LineWidth', 1);
% ǰ��
h_front = plot(ransac_x, front_y, 'black-', 'LineWidth', 1);
% ͣ����
h_stop_line = plot(ransac_x, stop_y, 'cyan-', 'LineWidth', 1);
% ͣ����
h_stop =plot(dest_x, dest_y, 'r*', 'MarkerSize', 8);
% �����״�λ��
h_laser = plot(0, 0, 'black*', 'MarkerSize', 8);

str = sprintf('i = %d', i);
text(100, 1000, str)
legend([h_stop h_laser h_raw h_right h_path h_front], ...
    '�յ�滮λ��', '�����״�λ��', '�����״�ԭʼ���', '����������', '�滮����·��', 'ǰ���������');

axis equal;
xlim([min(total_x), max(total_x)]);
ylim([min(total_y), max(total_y)]);

hold off;
pause(0.08);clf;


