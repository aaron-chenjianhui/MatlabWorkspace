function [ angle_seq_out, lidar_data_out ] = DataFilter( angle_seq, lidar_data, filter_angle_range, filter_data_range )
%DataFilter 
%   

% lidar_data_range = [0.8, 10];
% lidar_data_narr_range = [0.8, 2];


angle_min = min(filter_angle_range);
angle_max = max(filter_angle_range);
data_min = min(filter_data_range);
data_max = max(filter_data_range);

angle_seq_out = [];
lidar_data_out = [];

for i = 1:size(lidar_data, 1)
    if (angle_seq(i) >= deg2rad(angle_max) || angle_seq(i) < deg2rad(angle_min))
       angle_seq_out = [angle_seq_out;angle_seq(i)];
       lidar_data_out = [lidar_data_out;lidar_data(i)];
    else
        if (lidar_data(i) < data_max && lidar_data(i) > data_min)
            angle_seq_out = [angle_seq_out;angle_seq(i)];
            lidar_data_out = [lidar_data_out;lidar_data(i)];
        end
    end
    
end

% for i = 1:num
%     if (angle_seq(i) > deg2rad(90) || angle_seq(i) < deg2rad(-90))
%         if (lidar_data(i)>lidar_data_narr_range(1) && lidar_data(i)<lidar_data_narr_range(2))
%             angle_seq_out_tmp = [angle_seq_out_tmp;angle_seq(i)];
%             lidar_data_out_tmp = [lidar_data_out_tmp;lidar_data(i)];
%             intensity_data_out_tmp = [intensity_data_out_tmp;intensity_data(i)];
%         end
%     else
%         if (lidar_data(i)>lidar_data_range(1) && lidar_data(i)<lidar_data_range(2))
%             angle_seq_out_tmp = [angle_seq_out_tmp;angle_seq(i)];
%             lidar_data_out_tmp = [lidar_data_out_tmp;lidar_data(i)];
%             intensity_data_out_tmp = [intensity_data_out_tmp;intensity_data(i)];
%         end
%     end 
% end
