% 读取雷达数据
% static_1/static_2/static_3/moving_1/moving_2
file_name = './data/lidar_data.txt';
[angle_seq, lidar_data] = ReadData(file_name);

data_num = size(lidar_data, 2);
angle_min = deg2rad(-90);
angle_max = deg2rad(90);
sample_interval = 2;

angle_select = [angle_min, angle_max];

angle_seq_select = [];
lidar_data_select = [];
for i = 1:data_num
    [angle_seq_now, lidar_data_now] = DataSelect(angle_select, angle_seq, lidar_data(:,i));
    angle_seq_select = angle_seq_now;
    lidar_data_select = [lidar_data_select lidar_data_now];
end


angle_samp = downsample(angle_seq_select, 2);
lidar_samp = downsample(lidar_data_select, 2);

num = size(angle_samp, 1);

angle_range = angle_max - angle_min;
angle_incre = angle_range/(num -1);

data_max = 30;
data_reso = 0.001;

fd = fopen('exp.log', 'w');

for i = 1:data_num
    %
    fprintf(fd, 'ROBOTLASER1 0 %.6f %.6f %.6f %.6f %.6f 0 %d ', ...
        angle_min, angle_range, angle_incre, data_max,...
        data_reso, num);
    
    %
    fprintf(fd, '%.3f ', lidar_samp(:,i));
    
    fprintf(fd, '0 -0.040001 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 0.000000 1211.520329 x 0.015885 \r\n');
    
end
