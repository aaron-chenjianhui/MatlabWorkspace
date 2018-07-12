function [ angle_seq_out, lidar_data_out ] = DataIgn( ign_angle, angle_seq, lidar_data )
%DataIgn Eliminate LiDAR data in ign_angle sequence
%

% if the difference of two double elements is less than DATASAMETHRE, 
% they will be considered the same
DATASAMETHRE = deg2rad(0.001);

ign_angle_max = max(ign_angle);
ign_angle_min = min(ign_angle);

angle_seq_out = [];
lidar_data_out = [];

ign_flag = 0;

for i = 1:size(angle_seq, 1)
    if (angle_seq(i) > ign_angle_max || angle_seq(i) < ign_angle_min)
        angle_seq_out = [angle_seq_out angle_seq(i)];
        lidar_data_out = [lidar_data_out lidar_data(i)];
    else
        % find the same element in ign_angle
        for j = 1:size(ign_angle, 1)
            if (abs(ign_angle(j) - angle_seq(i)) < DATASAMETHRE)
                ign_flag = 1;
                ign_angle(j) = [];
                break;
            end
        end
        
        % there is no same element in ign_angle
        if (0 == ign_flag)
            angle_seq_out = [angle_seq_out angle_seq(i)];
            lidar_data_out = [lidar_data_out lidar_data(i)];
        else
            
            ign_flag = 0;
        end
    end
    
end

