function [ mat ] = Euler2Trans( x, y, theta )
%Euler2Trans 
%   this function only suitable for planar situation

mat(1,1) = cos(theta);
mat(1,2) = -sin(theta);
mat(1,3) = x;
mat(2,1) = sin(theta);
mat(2,2) = cos(theta);
mat(2,3) = y;
mat(3,1) = 0;
mat(3,2) = 0;
mat(3,3) = 1;

end

