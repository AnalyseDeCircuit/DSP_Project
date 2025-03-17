%% 语音增强系统 - 谱减法实现
clear; clc; close all;

%% 1. 读取带噪语音文件
[input_signal, fs] = audioread('noisy_speech.wav'); % 替换为实际采集的语音文件
input_signal = input_signal(:,1); % 取单声道
t = (0:length(input_signal)-1)/fs;

%% 2. 预加重处理（提升高频）
a = 0.95; % 预加重系数
pre_emph = [1, -a];
signal_pre = filter(pre_emph, 1, input_signal);

%% 3. 分帧参数设置
frame_length = 256;       % 帧长（通常20-30ms）
overlap = 128;           % 帧重叠
win = hamming(frame_length); % 汉明窗

%% 4. 分帧处理
frames = buffer(signal_pre, frame_length, overlap, 'nodelay');
num_frames = size(frames, 2);

%% 5. FFT参数
nfft = 2^nextpow2(frame_length);
noise_est_frames = 5;    % 用于噪声估计的帧数

%% 6. 初始化噪声功率谱估计
mag_noise = zeros(nfft,1);
for k = 1:noise_est_frames
    frame = frames(:,k).*win;
    mag_noise = mag_noise + abs(fft(frame, nfft));
end
mag_noise = mag_noise/noise_est_frames;

%% 7. 谱减法处理
enhanced_frames = zeros(size(frames));
alpha = 1.5;  % 过减因子
beta = 0.02;  % 谱下限系数

for i = 1:num_frames
    % 加窗处理
    frame = frames(:,i).*win;
    
    % FFT变换
    spec = fft(frame, nfft);
    mag = abs(spec);
    phase = angle(spec);
    
    % 谱减处理
    mag_enhanced = mag - alpha*mag_noise;
    mag_enhanced = max(mag_enhanced, beta*mag_noise);
    
    % 合成相位
    enhanced_spec = mag_enhanced .* exp(1i*phase);
    
    % IFFT变换
    enhanced_frame = real(ifft(enhanced_spec, nfft));
    
    % 取前frame_length点并加窗
    enhanced_frames(:,i) = enhanced_frame(1:frame_length).*win;
end

%% 8. 重叠相加合成信号
enhanced_signal = overlap_add(enhanced_frames, overlap);

%% 9. 去加重处理
de_emph = filter(1, pre_emph, enhanced_signal);

%% 10. 结果对比
% 绘制波形
figure;
subplot(211); plot(t, input_signal); title('原始带噪语音');
subplot(212); plot(t(1:length(de_emph)), de_emph); title('增强后语音');

% 播放音频
disp('播放原始语音...');
soundsc(input_signal, fs);
pause(length(input_signal)/fs + 1);
disp('播放增强后语音...');
soundsc(de_emph, fs);

% 保存结果
audiowrite('enhanced_speech.wav', de_emph, fs);

%% 重叠相加函数
function signal = overlap_add(frames, overlap)
    [frame_length, num_frames] = size(frames);
    step = frame_length - overlap;
    signal_length = frame_length + (num_frames-1)*step;
    signal = zeros(signal_length, 1);
    
    for i = 1:num_frames
        start = (i-1)*step + 1;
        signal(start:start+frame_length-1) = ...
            signal(start:start+frame_length-1) + frames(:,i);
    end
end