load('1.mat');
load('2.mat');
load('3.mat');
load('4.mat');
load('5.mat');
load('6.mat');
load('7.mat');

comp = [comp_1/2 comp_2/2 comp_3/2 comp_4/2 comp_5/2 comp_front_1 comp_front_2];
disp('The compensating value of lidar is');
disp(mean(comp));

% the compensating value of lidar if 32.5 mm