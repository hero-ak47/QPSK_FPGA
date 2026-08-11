
clc; clear; close all
%% SECTION 1: BẮT ĐẦU THU TÍN HIỆU
% disp('Đang ghi âm... Hãy phát tín hiệu ở máy kia.');
% r = audiorecorder(48000, 24, 1);
% record(r);

%% SECTION 2: DỪNG THU VÀ XỬ LÝ TÍN HIỆU
% stop(r);
% disp('Đã dừng ghi âm. Đang xử lý tín hiệu...');
%y = getaudiodata(r, 'double');
clc; clear; close all;
% 1. THÔNG SỐ VÀ TÁI TẠO PREAMBLE LÀM CHUẨN MẪU
Fs = 48000; Fc = 10000; 
Rs = 500;   % bit rate
L = Fs/Rs;  % numbers of sample in 1s

% RRC
beta = 0.5; span = 6;
h_rrc = rcosdesign(beta, span, L, 'sqrt');

% Zadoff-Chu
Nzc = 63;
u   = 25;
Lsym = Nzc * L;              % Số mẫu của MỘT khối preamble ZC sau pulse shaping
n   = (0:Nzc-1).';
pss = exp(-1j*pi*u*n.*(n+1)/Nzc);

%% ================= NẠP TÍN HIỆU =================
[y, Fs_read] = audioread('tx_signal_1.wav');
if Fs_read ~= Fs
    warning('Fs trong file (%d) khác Fs khai báo (%d)!', Fs_read, Fs);
end
disp('Đã nạp tín hiệu từ tx_signal_1.wav');

% Khu offset
y = y - mean(y);

rayleighChan = comm.RayleighChannel( ...
    'SampleRate', 96000, ...
    'PathDelays', [0 0.001 0.002], ...
    'AveragePathGains', [0 -3 -5], ...
    'MaximumDopplerShift', 2);
y = rayleighChan(y);

% mo phong kenh truyen : CFO + Fadding +AWGN
CFO_true = 4;  SNR_dB = -1;
h_ch_audio = [1; -2 + 1; 9 + 1j];
y = conv(y, h_ch_audio);
y = y .* exp(1j*2*pi*CFO_true*(0:length(y)-1)'/Fs);
y = my_awgn(y, SNR_dB);
plot(y);

% preable laf ZC de dong bo
preamble_sym_rx = pss;
preamble_bb_ideal = upfirdn(preamble_sym_rx, h_rrc, L);

% 2. TIỀN XỬ LÝ: LỌC BĂNG THÔNG & HẠ TẦN XUỐNG BASEBAND
[b, a] = butter(4, [9000 11000]/(Fs/2), 'bandpass');
y_filt = filter(b, a, y);

t_rx = (0:length(y_filt)-1)' / Fs;
y_bb = y_filt .* exp(-1j * 2 * pi * Fc * t_rx);

y_mf = conv(y_bb, h_rrc, 'same');

% 3. ĐỒNG BỘ KHUNG (FRAME SYNCHRONIZATION)
[xc, lags] = xcorr(y_mf, preamble_bb_ideal);
[peak_val, max_idx] = max(abs(xc));
start_sample = lags(max_idx) + 1;

noise_floor = median(abs(xc));
fprintf('\n--- DEBUG: FRAME SYNC ---\n');
fprintf('start_sample = %d (length y_mf = %d)\n', start_sample, length(y_mf));
fprintf('Peak/Noise = %.2f\n', peak_val/noise_floor);
if peak_val/noise_floor < 5
    warning('Peak/Noise thấp -> có thể sync SAI vị trí!');
end

Nguard   = 14;    % khoảng lặng 
Npayload = 500;   % pilot + data + pilot + data + ....
skip = Nzc + Nguard;              % = 77
total_sym_len = skip + Npayload;  % = 577  --độ dài toàn bộ bản tin: preamble + guard + pilot + data + pilot + data + ....

if start_sample < 1 || (start_sample + total_sym_len*L) > length(y_mf)
    error('Không tìm thấy khung tín hiệu nguyên vẹn. Hãy thử thu phát lại!');
end

% 4. TRÍCH XUẤT SYMBOL (DOWNSAMPLING) -- lấy mẫu đại diện cho các symbol IQ
rx_symbols = zeros(total_sym_len, 1);

first_sym_offset = (span*L/2) + 1;  % lấy mẫu ở chính giữa symbol

for k = 1:total_sym_len
    idx = start_sample + first_sym_offset + (k-1)*L - 1;
    rx_symbols(k) = y_mf(idx);
end

rx_payload = rx_symbols(skip+1:end); % Bỏ qua preamble + guard  -> thu được các symbol pilot + data

% ---- Vị trí pilot & chuỗi pilot ZC (khai báo TRƯỚC để dùng cho cả CFO và RLS) ----
pilot_indices = 1:5:500;             % 100 vị trí pilot
data_indices  = setdiff(1:500, pilot_indices); % hàm setdiff trả về hiệu của 2 tập hợp -> index của data là các số còn lại từ 1 -> 500

N_pilots = length(pilot_indices);    % = 100 pilot symbols
u_p = 7;
n_p = (0:N_pilots-1).';
zc_pilots = sqrt(2) * exp(-1j * pi * u_p * n_p .* (n_p + 1) / N_pilots);

%-------------------------------------------------------------------------------------------------------------------------------
% 5. ƯỚC LƯỢNG & BÙ CFO BẰNG LEAST-SQUARES TRÊN PHA PILOT ZC
%-------------------------------------------------------------------------------------------------------------------------------
rx_pilots_raw = rx_payload(pilot_indices);

% So pha pilot thu được với pilot ZC lý tưởng tương ứng 
% vì pilot là chuỗi Zadoff-Chu, mỗi vị trí có pha riêng

pilot_phase_raw = angle(rx_pilots_raw ./ zc_pilots);

fprintf('\n--- DEBUG: PILOT PHASE (TRƯỚC UNWRAP) ---\n');
disp(pilot_phase_raw(1:min(10,end)).');

% hàm angle chỉ trả về pha từ [-pi:pi] -> dùng hàm unwrap để bù k2pi -> mảng lưu pha các mẫu IQ sẽ tăng mượt nếu ảnh hưởng bời CFO
pilot_phase_unwrapped = unwrap(pilot_phase_raw);

% idx_col: vị trí symbol (0-based) của từng pilot TRONG PAYLOAD, dùng làm
% biến "thời gian" cho phép fit tuyến tính pha theo symbol index

% ma tran thiet ke A = [1 n] vs phi_CFO = A.[a b]
idx_col = pilot_indices(:) - 1;
A = [ones(length(idx_col),1), idx_col];

% tinh nghiem toi uu theo LS : phi = (A^T.A)^-1.A^T. phi_unwrap
coeffs = A \ pilot_phase_unwrapped;
slope  = coeffs(2);

Tsym = L / Fs;
cfo_est = slope / (2*pi*Tsym);

%------------------ debug----------------------
fprintf('\n--- DEBUG: CFO ESTIMATION ---\n');
fprintf('slope (rad/symbol) = %.6f\n', slope);
fprintf('CFO ước lượng      = %.4f Hz (CFO thật = %.4f Hz)\n', cfo_est, CFO_true);

phase_fit = A * coeffs;
residual = pilot_phase_unwrapped - phase_fit;
fprintf('RMS residual sau LS fit = %.6f rad\n', sqrt(mean(residual.^2)));
%---------------------------------------------------

% ---- Bù CFO cho toàn bộ 500 symbol payload ----
n_idx = (0:499).';
cfo_correction = exp(-1j * 2*pi * cfo_est * Tsym * n_idx);
rx_payload_cfo = rx_payload .* cfo_correction;
%---------------------------------------------------

figure;
scatter(real(rx_payload_cfo), imag(rx_payload_cfo), 'b.'); hold on;
scatter([1 -1 1 -1], [1 1 -1 -1], 'rx', 'LineWidth', 2);
title('Chòm sao trước rls (Đã bù CFO)');
xlabel('In-phase'); ylabel('Quadrature');
grid on; axis square;

%----------------------------------------------------------------
% 6. CÂN BẰNG KÊNH BẰNG RLS (DÙNG PILOT ZC) - CHẠY TRÊN rx_payload_cfo
%----------------------------------------------------------------
lambda = 0.1;       % Hệ số quên (giai đoạn warm-up) (lMBDA CANG lon thi cang nho lau)
P      = 100;       % 
W      = 1 + 0*1j;  % trọng số cân bằng ban đầu

rx_payload_eq = zeros(500, 1);
W_track       = zeros(500, 1);

% ---- Warm-up tại pilot ZC đầu tiên để hội tụ nhanh ----
first_pilot_rx = rx_payload_cfo(pilot_indices(1));
first_pilot_tx = zc_pilots(1);
for it = 1:20
    y_wu = first_pilot_rx;
    d_wu = first_pilot_tx;

    y_mag_sq = real(y_wu)^2 + imag(y_wu)^2;
    k = (P * conj(y_wu)) / (lambda + P * y_mag_sq);
    e = d_wu - W * y_wu;
    W = W + k * e;
    P = (P - k * y_wu * P) / lambda;
end

% ---- RLS liên tục qua toàn bộ payload đã bù CFO (Decision-Directed + ZC Pilot) ----
lambda = 0.9;        % Hệ số quên chính thức cho vòng lặp chính
pilot_cnt = 0;

for i = 1:500
    y_k = rx_payload_cfo(i);   % <--  tín hiệu ĐÃ BÙ CFO

    % 1. Lấy dữ liệu tham chiếu (Reference)
    if ismember(i, pilot_indices)   % nếu đâng xét đến symbl pilot
        pilot_cnt = pilot_cnt + 1;
        d_k = zc_pilots(pilot_cnt); % d_k là pilot tham chiếu (đã biết) cho symbol tiếp theo

    else                            % nếu đang xét đến symbol data
        eq_temp = W * y_k;          % cập nhật RLS cho symbol data
        d_k = sign(real(eq_temp)) + 1j * sign(imag(eq_temp)); % lại thành symbol tham chiếu (dự đoán)
    end

    % 2. Cân bằng tín hiệu nhận được, lưu vào mảng kết quả
    rx_payload_eq(i) = W * y_k;

    % 3. Cập nhật RLS cho symbol tiếp theo
    y_mag_sq = real(y_k)^2 + imag(y_k)^2;
     % độ lợi kalman : quyết định tin sai số e đến mức nào
    k = (P * conj(y_k)) / (lambda + P * y_mag_sq);
    % Sai số ước lượng
    e = d_k - rx_payload_eq(i);
    % Cập nhật trọng số RLS mới
    W = W + k * e;
    % Cập nhật P: P càng lớn, thuật toán càng tự tin cập nhật mạnh -> hội tụ nhanh nhưng dễ dao dộng
    P = (P - k * y_k * P) / lambda;

    W_track(i) = W;  % lưu lại các trọng số để plot
end

rx_data_eq = rx_payload_eq(data_indices);
%----------------------------------------------------------------

figure;
scatter(real(rx_data_eq), imag(rx_data_eq), 'b.'); hold on;
scatter([1 -1 1 -1], [1 1 -1 -1], 'rx', 'LineWidth', 2);
title('Chòm sao trước xoay pha (Đã bù CFO + cân bằng RLS bằng ZC Pilot)');
xlabel('In-phase'); ylabel('Quadrature');
grid on; axis square;

% ---- Bù lệch pha dư (residual phase offset) ----
constellation = [pi/4, 3*pi/4, -3*pi/4, -pi/4];
rx_phase = angle(rx_data_eq(1:400));
delta_rx = zeros(1,400);
for i = 1:400
    phase_error = angle(exp(1j*(rx_phase(i) - constellation)));
    [~, idx] = min(abs(phase_error));
    delta_rx(i) = phase_error(idx);
end
phi = angle(mean(exp(1j*delta_rx)));
rx_data_eq = rx_data_eq .* exp(-1j*phi);

fprintf('\nResidual phase offset sau RLS = %.4f rad (%.2f do)\n', phi, rad2deg(phi));

% ---- Kiểm tra hội tụ: vẽ |W| và pha theo symbol index ----
figure;
subplot(2,1,1);
plot(abs(W_track), 'LineWidth', 1.5); grid on;
title('Bien do trong so |W| cap nhat qua 500 symbol (sau bu CFO)');
xlabel('Symbol index'); ylabel('|W|');

subplot(2,1,2);
plot(rad2deg(angle(W_track)), 'LineWidth', 1.5); grid on;
title('Pha cua W cap nhat qua 500 symbol (sau bu CFO)');
xlabel('Symbol index'); ylabel('Pha (do)');

% 7. GIẢI ĐIỀU CHẾ QPSK (DEMAPPING) VÀ SO SÁNH
rx_bits = zeros(length(rx_data_eq)*2, 1);
rx_bits(1:2:end) = real(rx_data_eq) < 0;
rx_bits(2:2:end) = imag(rx_data_eq) < 0;

rng(123);
tx_data_bits_ideal = randi([0 1], 400 * 2, 1);

num_errors = sum(rx_bits ~= tx_data_bits_ideal);
BER = num_errors / length(tx_data_bits_ideal);

% 8. HIỂN THỊ KẾT QUẢ
disp('--- KẾT QUẢ TRUYỀN NHẬN (Bu CFO (LS pilot ZC) + Can bang RLS) ---');
fprintf('Số bits truyền: %d\n', length(tx_data_bits_ideal));
fprintf('Số bits lỗi: %d\n', num_errors);
fprintf('Tỉ lệ lỗi bit (BER): %f\n', BER);

figure;
scatter(real(rx_data_eq), imag(rx_data_eq), 'b.'); hold on;
scatter([1 -1 1 -1], [1 1 -1 -1], 'rx', 'LineWidth', 2);
title('Chòm sao tín hiệu nhận được (Bù CFO + Cân bằng RLS bằng ZC Pilot)');
xlabel('In-phase'); ylabel('Quadrature');
grid on; axis square;

%%
function Y = my_awgn(X, SNR_dB)
% Hàm giả lập awgn(X, SNR_dB, 'measured') cho tài khoản Basic

P_signal = mean(abs(X).^2);
SNR_linear = 10^(SNR_dB / 10);
P_noise = P_signal / SNR_linear;

if isreal(X)
    noise = sqrt(P_noise) * randn(size(X));
else
    noise = sqrt(P_noise/2) * (randn(size(X)) + 1i*randn(size(X)));
end

Y = X + noise;
end
