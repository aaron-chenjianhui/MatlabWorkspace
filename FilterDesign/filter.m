clear;
%% 读取数据
% 读取滤波器参数
% [Az, Bz] = Butterworth_filter(0.08, 5);
% % load('./FilterData/coeff_0.004.mat');

Wp = 0.1/25;
Ws = 10/25;
Rp = 0.1737;
Rs = 60;
% 求巴特沃斯阶数和3db截至角频率
[n,Wn] = buttord(Wp,Ws,Rp,Rs);
% 求传递函数
[b,a] = butter(n,Wn);

% 读取雷达数据
% static_1/static_2/static_3/moving_1/moving_2
file_name = './Data/.txt';
[angle_seq, lidar_data] = ReadData(file_name);

% file_name1 = './FilterData/custom_data.txt';
% [angle_seq_1, lidar_data_1] = ReadData(file_name1);
% file_name2 = './FilterData/lidar_data.txt';
% [angle_seq_2, lidar_data_2] = ReadData(file_name2);
% lidar_filter = lidar_data_1(1,:);
% lidar_raw = lidar_data_2(1,:);

% 选择某个角度的数据进行滤波
seq_index = 170;
% 采样周期
time_interval = 1/50;
% 单位转换（m -> mm）
lidar_data_select = lidar_data(seq_index, :)*1000;
lidar_data_select_x = lidar_data(seq_index, :)*cos(angle_seq(seq_index))*1000;
lidar_data_select_y = lidar_data(seq_index, :)*sin(angle_seq(seq_index))*1000;
% 采样的数据量
num = size(lidar_data_select, 2);

time = linspace(0, time_interval*(num-1), num);

% 显示原始数据
figure('NumberTitle', 'off', 'Name', 'Lidar Data');
plot(time, lidar_data_select_x, 'g');hold on;grid on;
xlabel('Time/s');
ylabel('Distance/mm');


%% 频谱分析

% % FFT过程
% hz = 50;
% % 确定FFT的采样数目
% N = 1;
% while(2^N<num)
%     N = N +1;
% end
% fft_num = 2^(N-1);
% % 确定FFT的采样点
% fft_processor = lidar_data_select;
% % 进行FFT
% fft_spec=fft(fft_processor(1:fft_num),fft_num);
% % 计算真实的幅值
% mag_fft = abs(fft_spec);
% mag_fft = mag_fft/(fft_num/2);
% mag_fft(1) = mag_fft(1)/2;
% % 计算采样频率
% freq_fft=(0:(fft_num-1))/fft_num*hz;
% % 绘制频谱图，只需绘制采样频率的一半
% figure('NumberTitle', 'off', 'Name', 'Raw Spectrum Map');
% plot(freq_fft(1:fft_num/2),mag_fft(1:fft_num/2), 'r');hold on;
% title('spectrum map');
% xlabel('frequency/Hz');
% ylabel('magnitude');



%% 滤波

% % 高斯滤波
% r = 7;
% sigma = 1;
% filter_out = Gaussianfilter(lidar_data_select, sigma, r);
% figure('NumberTitle', 'off', 'Name', 'Lidar Data');
% plot(time, lidar_data_select, 'g');hold on;
% plot(time, filter_out, 'r');
% xlabel('Time/s');
% ylabel('Distance/mm');


% 滑动平均滤波
filter_num = 200;
for i=1:filter_num-1
    ave_filter_out(i) = sum(lidar_data_select(1:i))/i;
end

for i=filter_num:num
    ave_filter_out(i) = lidar_data_select(i);
    filter_sum = sum(lidar_data_select((i-filter_num+1):i));
    ave_filter_out(i) = filter_sum/filter_num;
end
% figure('NumberTitle', 'off', 'Name', 'Lidar Data');
% plot(time, lidar_data_select, 'g');hold on;
% plot(time, ave_filter_out, 'r');
% xlabel('Time/s');
% ylabel('Distance/mm');


% % 滤波结束后的频谱分析
% hz = 50;
% % 确定FFT的采样数目
% N = 1;
% while(2^N<num)
%     N = N +1;
% end
% fft_num = 2^(N-1);
% % 确定FFT的采样点
% fft_processor = filter_out;
% % 进行FFT
% fft_spec=fft(fft_processor(1:fft_num),fft_num);
% % 计算真实的幅值
% mag_fft = abs(fft_spec);
% mag_fft = mag_fft/(fft_num/2);
% mag_fft(1) = mag_fft(1)/2;
% % 计算采样频率
% freq_fft=(0:(fft_num-1))/fft_num*hz;
% % 绘制频谱图，只需绘制采样频率的一半
% figure('NumberTitle', 'off', 'Name', 'Raw Spectrum Map');
% plot(freq_fft(1:fft_num/2),mag_fft(1:fft_num/2), 'r.');hold on;
% title('spectrum map');
% xlabel('frequency/Hz');
% ylabel('magnitude');


% Butterworth
% 采用设计好的参数Az，Bz进行滤波
% filter_out = BwFilter(lidar_data_select, Az, Bz);
filter_out = filter(b, a, lidar_data_select);

% filter_out_x = ave_filter_out*cos(angle_seq(seq_index));
% filter_out_y = ave_filter_out*sin(angle_seq(seq_index));
% subplot(1,2,1);plot(time, filter_out_x, 'r');
% subplot(1,2,2);plot(time, filter_out_y, 'b');

figure('NumberTitle', 'off', 'Name', 'Lidar Data');
plot(time, lidar_data_select, 'g');hold on;
plot(time, ave_filter_out, 'y');grid on;
plot(time, filter_out, 'r');
xlabel('Time/s');
ylabel('Distance/mm');

