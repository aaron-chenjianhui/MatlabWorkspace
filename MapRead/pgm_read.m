clc,clear,close all;
map = imread('./MapData/new_large_map.pgm');
[row col] = size(map);

%%
rot_map = imrotate(map, -1.8, 'bilinear', 'crop');
new_rot_map = uint8(205*ones(row, col));


figure(1);
imshow(rot_map);

useful_mask = roipoly(rot_map);
new_rot_map(useful_mask) = rot_map(useful_mask);
imshow(new_rot_map);




%%
new_map = new_rot_map;

figure(2);
imshow(new_rot_map);

free_mask = roipoly(new_rot_map);
new_map(free_mask) = uint8(254);

imshow(new_map);
imwrite(new_map, 'my_map.pgm');



