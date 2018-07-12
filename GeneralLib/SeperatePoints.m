function [ x_outer, y_outer, x_inner, y_inner ] = SeperatePoints( x, y, k, b )
%SeperatePoints 
%   

x_line = x;
y_line = k*x + b;

x_outer = [];
y_outer = [];
x_inner = [];
y_inner = [];

for i = 1:size(x, 1)
    if (y(i) > y_line(i))
        x_outer = [x_outer;x(i)];
        y_outer = [y_outer;y(i)];
    else
        x_inner = [x_inner;x(i)];
        y_inner = [y_inner;y(i)];
    end
end

