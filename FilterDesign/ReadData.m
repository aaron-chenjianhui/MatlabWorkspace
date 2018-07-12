function [ angle_seq, lidar_data ] = ReadData( file_name )
%ReadData �˴���ʾ�йش˺����ժҪ
%   �˴���ʾ��ϸ˵��


gen_fid=fopen(file_name,'rt');

pack_count = 0;

head_flag = 0;
data_flag = 0;
tail_flag = 0;

angle_min = -2.35619;
angle_max = 2.35619;
angle_increment = 0.00872665;
time_increment = 2.77778e-05;
scan_time = 0.02;
range_min = 0.01;
range_max = 20;
lidar_data = [];
lidar_data_coor_x = [];
lidar_data_coor_y = [];

while (~feof(gen_fid))
   line = fgetl(gen_fid);


       switch(line)
           case '[seq]'
               head_flag = 1;
           case '[angle_min]'
               angle_min = sscanf(fgetl(gen_fid), '%f');
           case '[angle_max]'
               angle_max = sscanf(fgetl(gen_fid), '%f');
           case '[angle_increment]'
               angle_increment = sscanf(fgetl(gen_fid), '%f');
           case '[ranges]'
               [range_data, range_count] = sscanf(fgetl(gen_fid), '%f,  ');
               data_flag = 1;
           case '[tail]'
               tail_flag = 1;
           otherwise
               %
       end


       if (tail_flag)
           if (head_flag && data_flag)
               pack_count = pack_count + 1;
               % ��ɽǶ�����
               data_num = round((angle_max-angle_min)/angle_increment) + 1;
               angle_seq = linspace(angle_min, angle_max, data_num)';
               % ��Ӧ�ļ����״��������
               lidar_data(:,pack_count) = range_data;
               % ת��Ϊֱ�����
               lidar_data_coor_x(:,pack_count) = range_data.*cos(angle_seq);
               lidar_data_coor_y(:,pack_count) = range_data.*sin(angle_seq);
           else
               disp('Data Package Broken');
           end
           tail_flag = 0;
       end
end

fclose(gen_fid);

end

