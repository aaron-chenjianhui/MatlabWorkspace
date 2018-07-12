%% Load data
clear;

% select left angle range
LEFT_MIN = deg2rad(45);
LEFT_MAX = deg2rad(135);

load('./data/inner.mat');
figure;
polar(angle, data, 'b.');

% left
[angle_left, data_left] = DataSelect([LEFT_MIN LEFT_MAX], angle, data);

figure;
polar(angle_left, data_left, 'b.');

%% global localization
[left_x, left_y] = Polar2Rec(angle_left, data_left);

[left_k, left_b] = FindLine(left_x, left_y);

x_min = min(left_x);
x_max = max(left_x);
x = x_min:0.1:x_max;
y = left_k*x + left_b;
figure;
plot(left_x, left_y, 'b.');hold on;axis equal;
plot(x, y, 'r');hold off;

%% select local detection angle range
LOC_MIN = deg2rad(55);
LOC_MAX = deg2rad(135);

%% 
% threshold parameters
RANK_THRE = -10;
MAX_ITER = 5;

% global location selection
[angle_select, data_select] = DataSelect([LOC_MIN LOC_MAX], angle, data);
[x_select, y_select] = Polar2Rec(angle_select, data_select);
x_min = min(x_select);
x_max = max(x_select);
x = x_min:0.1:x_max;
y = left_k*x + left_b;

figure;
% polar(angle_select, data_select, 'b.');
plot(x_select, y_select, 'b.');hold on;axis equal;
plot(x, y, 'r');

[x_outer, y_outer, x_inner, y_inner] = SeperatePoints(x_select, y_select, left_k, left_b);
plot(x_outer, y_outer, 'g.');
plot(x_inner, y_inner, 'k.');

figure;
for i = 1:MAX_ITER
    x_local = x_inner;
    y_local = y_inner;
    [k_local, b_local, err, rank] = FindLine(x_local, y_local);
    if (rank > RANK_THRE)
        break;
    end
    
    [x_outer, y_outer, x_inner, y_inner] = SeperatePoints(x_local, y_local, k_local, b_local);
    
    % plot
    x_min = min(x_local);
    x_max = max(x_local);
    x = x_min:0.1:x_max;
    y = k_local*x + b_local;
    
    plot(x_local, y_local, 'b.');hold on;axis equal;
    plot(x, y, 'r');
    plot(x_outer, y_outer, 'g.');
    plot(x_inner, y_inner, 'k.');
    clf;
end

figure;
x_min = min(x_select);
x_max = max(x_select);
x = x_min:0.1:x_max;
y = k_local*x + b_local;

plot(left_x, left_y, 'b.');hold on;axis equal;
plot(x, y, 'r');

% [x, y] = Polar2Rec(angle, data);
% plot(x, y, '.', 'markersize', 5);hold on;
% plot(0, 0, 'ro', 'markersize', 8);