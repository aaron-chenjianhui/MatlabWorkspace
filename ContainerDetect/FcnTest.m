clear;
file_name = './Data/lidar_data_1.txt';
[angle_seq, lidar_data] = ReadData(file_name);

num = size(lidar_data, 2);

for i = 600:100:num
    pack_select = i;
    
    angle_seq_select = angle_seq;
    lidar_data_select = lidar_data(:,pack_select);
    
    [path_k, path_b, dest_x, dest_y, dest_theta, flag] = LidarDataFcn(angle_seq_select, lidar_data_select);
    
    total_x = lidar_data_select.*cos(angle_seq_select)*1000;
    total_y = lidar_data_select.*sin(angle_seq_select)*1000;
    ransac_x = linspace(min(total_x), max(total_x), 500);
    path_y = ransac_x*path_k + path_b;
      
    hold on
    % Raw Data
    h_raw = plot(total_x, total_y, 'b.', 'MarkerSize', 5);
    % 路径
    h_path = plot(ransac_x, path_y, 'cyan--', 'LineWidth', 1);
    % 停车点
    h_stop =plot(dest_x, dest_y, 'r*', 'MarkerSize', 8);
    % 激光雷达位置
    h_laser = plot(0, 0, 'black*', 'MarkerSize', 8);
    
    str = sprintf('i = %d', i);
    text(100, 1000, str)
%     legend([h_stop h_laser h_raw h_right h_path h_front], ...
%         '终点规划位置', '激光雷达位置', '激光雷达原始数据', '侧壁拟合曲线', '规划中线路径', '前壁拟合曲线');
    
    axis equal;
    xlim([min(total_x), max(total_x)]);
    ylim([min(total_y), max(total_y)]);
    
    hold off;
    pause(0.08);clf;
end