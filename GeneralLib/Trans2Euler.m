function [ x, y, theta ] = Trans2Euler( mat )
%Trans2Euler 
%   this function only suitable for planar situation

x = mat(1,3);
y = mat(2,3);

theta = atan2(mat(2,1), mat(1,1));

end

