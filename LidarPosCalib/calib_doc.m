clear;
plc_data;
laser_data;

width = 2350;


% for i = 1:6
%     x_lidar = data(i,1);
%     y_lidar = data(i,2);
%     theta_lidar = data(i,3);
%     
%     theta_plc = data(i,6);
%     x_plc = data(i,4)*cos(abs(theta_plc));
%     y_plc = data(i,5)*cos(abs(theta_plc));
%     
%     
%     T_laser_in_ori = Euler2Trans(x_lidar, y_lidar, theta_lidar);
%     T_base_in_ref = Euler2Trans(-y_plc, -x_plc, -theta_plc);
%     
%     T_laser_in_base = T_base_in_ref*inv(T_ori_in_ref)*inv(T_laser_in_ori);
%     [x, y, theta] = Trans2Euler(T_laser_in_base);
% end

i = 1;

lidar_data = ori_data(i,:);
x_lidar = lidar_data(1);
y_lidar = lidar_data(2);
theta_lidar = lidar_data(3);


x_plc = x_plc_data(i);
y_plc = y_plc_data(i);
theta_plc = theta_plc_data(i);

T_laser_in_ori = Euler2Trans(x_lidar, y_lidar, theta_lidar);
T_base_in_ori = Euler2Trans(x_plc, y_plc, theta_plc);

T_base_in_laser = inv(T_laser_in_ori)*T_base_in_ori;
T_laser_in_base = inv(T_base_in_laser);
[x, y, theta] = Trans2Euler(T_laser_in_base);
