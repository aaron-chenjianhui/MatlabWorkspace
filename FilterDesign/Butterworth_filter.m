function [ Az, Bz ] = Butterworth_filter( p, s )


%% Butterworth滤波器的设计
% clear;
% 采样频率
fs = 50;
% 通带截至频率
% Wp = 2*pi*0.025;
Wp = 2*pi*p;
% 阻带截至频率
% Ws = 2*pi*4;
Ws = 2*pi*s;
% 通带最大衰减
% Rp = 3;
Rp = 3;
% 阻带最大衰减
% Rs = 40;
Rs = 40;

Wp = 0.1/25;
Ws = 10/25;
Rp = 0.1737;
Rs = 60;

% 求巴特沃斯阶数和3db截至角频率
[n,Wn] = buttord(Wp,Ws,Rp,Rs,'s');
% 求传递函数
[b,a] = butter(n,Wn,'s');
% 求零极点和增益
% [z,p,k] = butter(n,Wn,'s');

% 绘图
w = linspace(0,5)*2*pi;
H = freqs(b,a,w);
magH = abs(H);
phaH = unwrap(angle(H));
% plot(w/(2*pi),20*log10(magH));
% title('Butterworth');
% xlabel('Hz');
% ylabel('dB');

% 模拟滤波器化为数字滤波器
[Bz,Az] = impinvar(b,a,fs);
% Bz = round(Bz,5);
% Az = round(Az,5);

% Z变换的具体形式
syms z k;
syms fra num;
fra = 0;
num = 0;
for i=1:(n+1)
    fra = Bz(i)*z^(n+1-i) + fra;
    num = Az(i)*z^(n+1-i) + num;
end

num
% fz = fra/num;
% fk = iztrans(fz,k);