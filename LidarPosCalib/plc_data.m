width = 1700;
length = 1900;
mea_len = 1610;

x_front = [298 304 296 328];
width_front = [2350 2349 2342 2352];
x_back = [319 305 294 314];
width_back = [2359 2358 2355 2360];

left = [2934 2452 1917 2979];
right = [2944 2472 1940 2945];

theta_tmp = (x_back - x_front)/mea_len;
theta_plc_data = atan(theta_tmp);

x_tmp = (left+right)/2 + length/2;
x_plc_data = -x_tmp.*cos(abs(theta_plc_data));

y_tmp = (x_front + x_back)/2 + width/2;
y_plc_data = -y_tmp.*cos(abs(theta_plc_data));
