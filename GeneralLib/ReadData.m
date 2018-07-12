function [ angle_seq, lidar_data, pack_count ] = ReadData( file_name )
%ReadData read data from user defined data type
%   [ angle_seq, lidar_data, pack_count ] = ReadData( file_name )
%   The data format is:
%   -----------------------
%   [seq]
%   seq data
%   [angle_min]
%   angle_min data
%   [angle_max]
%   angle_max data
%   [angle_increment]
%   angle_increment data
%   [time_increment]
%   time_increment data
%   [scan_time]
%   scan_time data
%   [range_min]
%   range_min data
%   [range_max]
%   range_max data
%   [ranges]
%   ranges data
%   [intensities]
%   intensities data
%   [tail]
%   -----------------------

% open file
gen_fid=fopen(file_name,'rt');

% the number of packages
pack_count = 0;

head_flag = 0;
data_flag = 0;
intensity_flag = 0;
tail_flag = 0;

% default value
seq = 0;
angle_min = -2.35619;
angle_max = 2.35619;
angle_increment = 0.00872665;
time_increment = 2.77778e-05;
scan_time = 0.02;
range_min = 0.01;
range_max = 20;
lidar_data = [];
intensity_data = [];
lidar_data_coor_x = [];
lidar_data_coor_y = [];

while (~feof(gen_fid))
    line = fgetl(gen_fid);
    
    % read the frame
    switch(line)
        case '[seq]'
            seq = sscanf(fgetl(gen_fid), '%d');
            head_flag = 1;
        case '[angle_min]'
            angle_min = sscanf(fgetl(gen_fid), '%f');
        case '[angle_max]'
            angle_max = sscanf(fgetl(gen_fid), '%f');
        case '[angle_increment]'
            angle_increment = sscanf(fgetl(gen_fid), '%f');
        case '[time_increment]'
            time_increment = sscanf(fgetl(gen_fid), '%f');
        case '[scan_time]'
            scan_time = sscanf(fgetl(gen_fid), '%f');
        case '[range_min]'
            range_min = sscanf(fgetl(gen_fid), '%f');
        case '[range_max]'
            range_max = sscanf(fgetl(gen_fid), '%f');
        case '[ranges]'
            [range_data, range_count] = sscanf(fgetl(gen_fid), '%f,  ');
            data_flag = 1;
        case '[intensities]'
            [intensity, intensity_count] = sscanf(fgetl(gen_fid), '%f, ');
            intensity_flag = 1;
        case '[tail]'
            tail_flag = 1;
        otherwise
            if (~isempty(line))
                disp('Unknown frame label');
            end
    end
    
    % parse the frame
    if (tail_flag && head_flag)
        % there is no LiDAR data
        if (~data_flag && ~intensity_flag)
            disp('Broken Package');
            return;
        end
        
        pack_count = pack_count + 1;
        data_num = round((angle_max-angle_min)/angle_increment) + 1;
        angle_seq = linspace(angle_min, angle_max, data_num)';
             
        if (head_flag && data_flag)
            % lidar data
            lidar_data(:,pack_count) = range_data;
            
            % lidar data in rectangular coordinate
            lidar_data_coor_x(:,pack_count) = range_data.*cos(angle_seq);
            lidar_data_coor_y(:,pack_count) = range_data.*sin(angle_seq);
        end
        
        if (intensity_flag)
            intensity_data(:, pack_count) = intensity;
        end
        
        head_flag = 0;
        data_flag = 0;
        intensity_flag = 0;
        tail_flag = 0;
    end
end

fclose(gen_fid);

end

