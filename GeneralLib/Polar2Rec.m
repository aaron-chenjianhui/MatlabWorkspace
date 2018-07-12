function [ x, y ] = Polar2Rec( angle_seq, lidar_data )
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明

x = lidar_data.*cos(angle_seq)*1000;
y = lidar_data.*sin(angle_seq)*1000;

end

