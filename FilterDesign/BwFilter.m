function [ data_out ] = BwFilter( data_in, Az, Bz )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

if (size(Az, 2) == size(Bz, 2))
    order = size(Az, 2)-1;
else
    disp('Error');
    return;
end

num = size(data_in, 2);



for i=1:order
   data_out(i) = mean(data_in(1:i)); 
end

for i=(order+1):num
    raw_sum = 0;
    filter_sum = 0;
    for j=1:order+1
        raw_sum = raw_sum + Bz(j)*data_in(i-j+1);
    end
    for j=1:order
        filter_sum = filter_sum + Az(j+1)*data_out(i-j);
    end
    data_out(i) = (raw_sum - filter_sum)/Az(1);
end

