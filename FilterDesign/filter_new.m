% A revised method for filter design
% 3 functions are used in this script: buttord, butter, filter

time_interval = 1/50;

% 采样的数据量
num = size(oridata, 1);

time = linspace(0, time_interval*(num-1), num)';





Wp = 0.1/25;
Ws = 10/25;
Rp = 0.1737;
Rs = 60;
% 求巴特沃斯阶数和3db截至角频率
[n,Wn] = buttord(Wp,Ws,Rp,Rs);
% 求传递函数
[b,a] = butter(n,Wn);

filter_out = filter(b, a, oridata);hold on;

plot(time, oridata(:,2), 'r');
plot(time, filter_out(:,2), 'g');