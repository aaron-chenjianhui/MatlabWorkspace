function [ x, y ] = Polar2Rec( angle_seq, lidar_data )
%UNTITLED2 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

x = lidar_data.*cos(angle_seq)*1000;
y = lidar_data.*sin(angle_seq)*1000;

end

