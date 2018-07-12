function [ angle_seq_out, lidar_data_out ] = DataSelect( angle_range, angle_seq, lidar_data )
%DataSelect Select Lidar data in angle_range
%   angle_range: range_min, range_max
%   

lidar_range = deg2rad([-135 135]);
% Input Checking
if (angle_range(1) < lidar_range(1) || angle_range(2) > lidar_range(2))
    angle_seq_out = [];
    lidar_data_out = [];
    disp('Input Error');
    return;
end

seq_index = find(angle_seq>=angle_range(1) & angle_seq<=angle_range(2));
angle_seq_out = angle_seq(min(seq_index):max(seq_index));
lidar_data_out = lidar_data(min(seq_index):max(seq_index));
% lidar_data_out = lidar_data_out';

end

