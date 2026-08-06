%% SECTION 1: BẮT ĐẦU THU TÍN HIỆU
disp('Đang ghi âm... Hãy phát tín hiệu ở máy kia.');
r = audiorecorder(96000, 24, 1);
record(r);

%% SECTION 2: DỪNG THU VÀ XỬ LÝ TÍN HIỆU
stop(r);
disp('Đã dừng ghi âm. Đang xử lý tín hiệu...');
y = getaudiodata(r, 'double');
plot(y);

% 1. THÔNG SỐ VÀ TÁI TẠO PREAMBLE LÀM CHUẨN MẪU
Fs = 96000; Fc = 10000; Rs = 50; L = Fs/Rs;
beta = 0.5; span = 6;
h_rrc = rcosdesign(beta, span, L, 'sqrt');

CFO_true = 0;  SNR_dB = 25;
h_ch_audio = 1;
y = conv(y, h_ch_audio);
y = y .* exp(1j*2*pi*CFO_true*(0:length(y)-1)'/Fs);


Nzc = 63;
u   = 25;
Lsym = Nzc * L;              % số mẫu của MỘT khối preamble ZC sau pulse shaping
n   = (0:Nzc-1).';
pss = exp(-1j*pi*u*n.*(n+1)/Nzc);

% Tái tạo lại chính xác chuỗi Preamble ở máy thu để cross-correlation
preamble_sym_rx = pss;
preamble_bb_ideal = upfirdn(preamble_sym_rx, h_rrc, L);

% 2. TIỀN XỬ LÝ: LỌC BĂNG THÔNG & HẠ TẦN XUỐNG BASEBAND
% Lọc Bandpass từ 11kHz đến 13kHz để loại bỏ tạp âm phòng học
[b, a] = butter(4, [9000 11000]/(Fs/2), 'bandpass');
y_filt = filter(b, a, y);

% Hạ tần số sóng mang (Down-conversion)
t_rx = (0:length(y_filt)-1)' / Fs;
y_bb = y_filt .* exp(-1j * 2 * pi * Fc * t_rx);

% Matched Filter (Lọc bằng đúng bộ lọc RRC đã dùng ở máy phát)
y_mf = conv(y_bb, h_rrc, 'same');

% 3. ĐỒNG BỘ KHUNG (FRAME SYNCHRONIZATION)
% Tương quan chéo (Cross-correlation) để tìm đỉnh
[xc, lags] = xcorr(y_mf, preamble_bb_ideal);
[~, max_idx] = max(abs(xc));
start_sample = lags(max_idx) + 1; % Điểm bắt đầu chính xác của khung

if start_sample < 1 || (start_sample + 531*L) > length(y_mf)
    error('Không tìm thấy khung tín hiệu nguyên vẹn. Hãy thử thu phát lại!');
end

% 4. TRÍCH XUẤT SYMBOL (DOWNSAMPLING)
Nguard = 14;
Npayload = 500;
skip = Nzc + Nguard;              % = 77
total_sym_len = skip + Npayload;   % = 577

rx_symbols = zeros(total_sym_len, 1);

% Delay của bộ lọc là span*L/2. Đỉnh xcorr bù đắp delay này rồi, 
% ta lấy mẫu ngay tại điểm tương quan cao nhất và nhảy từng khoảng L
first_sym_offset = (span*L/2) + 1; 

for k = 1:total_sym_len
    idx = start_sample + first_sym_offset + (k-1)*L - 1;
    rx_symbols(k) = y_mf(idx);
end

% 5. ƯỚC LƯỢNG KÊNH VÀ CÂN BẰNG (CHANNEL ESTIMATION & EQUALIZATION)
rx_payload = rx_symbols(skip+1:end); % Bỏ qua 31 preamble

pilot_indices = 1:10:500;
data_indices = setdiff(1:500, pilot_indices);

% Rút trích các Pilots nhận được
rx_pilots = rx_payload(pilot_indices);
pilot_sym_ideal = 1 + 1j;

% Ước lượng độ lệch kênh tại các điểm có Pilot: H = Y / X
H_est = rx_pilots ./ pilot_sym_ideal;

% Nội suy kênh truyền tuyến tính cho các symbol dữ liệu nằm giữa các Pilot
H_interp = interp1(pilot_indices, H_est, 1:500, 'linear', 'extrap').';

% Cân bằng kênh (Zero-Forcing Equalizer)
rx_payload_eq = rx_payload ./ H_interp;
rx_data_eq = rx_payload_eq(data_indices); % Rút trích chỉ lấy Data

% 6. GIẢI ĐIỀU CHẾ QPSK (DEMAPPING) VÀ SO SÁNH
% Giải mã: Phần thực âm -> bit 1, Phần ảo âm -> bit 1
rx_bits = zeros(length(rx_data_eq)*2, 1);
rx_bits(1:2:end) = real(rx_data_eq) < 0;
rx_bits(2:2:end) = imag(rx_data_eq) < 0;

% Tái tạo lại data_bits ở máy phát để tính tỉ lệ lỗi (BER)
rng(123); % Cùng seed
tx_data_bits_ideal = randi([0 1], 450 * 2, 1);

% Tính Bit Error Rate
num_errors = sum(rx_bits ~= tx_data_bits_ideal);
BER = num_errors / length(tx_data_bits_ideal);

% 7. HIỂN THỊ KẾT QUẢ
disp('--- KẾT QUẢ TRUYỀN NHẬN ---');
fprintf('Số bits truyền: %d\n', length(tx_data_bits_ideal));
fprintf('Số bits lỗi: %d\n', num_errors);
fprintf('Tỉ lệ lỗi bit (BER): %f\n', BER);

figure;
scatter(real(rx_data_eq), imag(rx_data_eq), 'b.'); hold on;
scatter([1 -1 1 -1], [1 1 -1 -1], 'rx', 'LineWidth', 2);
title('Chòm sao tín hiệu nhận được (Đã qua cân bằng kênh)');
xlabel('In-phase'); ylabel('Quadrature');
grid on; axis square;
