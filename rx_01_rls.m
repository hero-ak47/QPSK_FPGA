%% SECTION 1: BẮT ĐẦU THU TÍN HIỆU
disp('Đang ghi âm... Hãy phát tín hiệu ở máy kia.');
r = audiorecorder(48000, 24, 1);
record(r);

%% SECTION 2: DỪNG THU VÀ XỬ LÝ TÍN HIỆU
stop(r);
disp('Đã dừng ghi âm. Đang xử lý tín hiệu...');
y = getaudiodata(r, 'double');
plot(y);

% 1. THÔNG SỐ VÀ TÁI TẠO PREAMBLE LÀM CHUẨN MẪU
Fs = 48000; Fc = 10000; Rs = 200; L = Fs/Rs;
beta = 0.5; span = 6;
h_rrc = rcosdesign(beta, span, L, 'sqrt');

% CFO_true = 0;  SNR_dB = 25;
% h_ch_audio = 1;
% y = conv(y, h_ch_audio);
% y = y .* exp(1j*2*pi*CFO_true*(0:length(y)-1)'/Fs);

Nzc = 63;
u   = 25;
Lsym = Nzc * L;              % số mẫu của MỘT khối preamble ZC sau pulse shaping
n   = (0:Nzc-1).';
pss = exp(-1j*pi*u*n.*(n+1)/Nzc);

% Tái tạo lại chính xác chuỗi Preamble ở máy thu để cross-correlation
preamble_sym_rx = pss;
preamble_bb_ideal = upfirdn(preamble_sym_rx, h_rrc, L);

% 2. TIỀN XỬ LÝ: LỌC BĂNG THÔNG & HẠ TẦN XUỐNG BASEBAND
% Lọc Bandpass từ 9kHz đến 11kHz để loại bỏ tạp âm phòng học
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

% 5. CÂN BẰNG KÊNH BẰNG RLS (TỐI ƯU HÓA: SCALAR & DECISION-DIRECTED)
rx_payload = rx_symbols(skip+1:end); % Bỏ qua preamble + guard
pilot_indices = 1:5:500;
data_indices  = setdiff(1:500, pilot_indices);
pilot_sym_ideal = 1 + 1j;   % pilot chuẩn đã biết trước

% ---- Tham số RLS ----
lambda = 0.97;     % Hệ số quên
P      = 100;       % Ma trận hiệp phương sai nghịch đảo (1-tap nên là số vô hướng)
W      = 1 + 0j;   % Trọng số cân bằng khởi tạo (W xấp xỉ 1/H)

rx_payload_eq = zeros(500, 1);
W_track       = zeros(500, 1);

% ---- Lặp "Warm-up" tại Pilot đầu tiên để hội tụ nhanh ----
first_pilot_rx = rx_payload(pilot_indices(1));
for it = 1:2
    y = first_pilot_rx;
    d = pilot_sym_ideal;
    
    % Toán học RLS vô hướng (Tối ưu tốc độ, bỏ qua ma trận/chuyển vị)
    y_mag_sq = real(y)^2 + imag(y)^2;
    k = (P * conj(y)) / (lambda + P * y_mag_sq); % Kalman gain
    e = d - W * y;                               % Sai số
    W = W + k * e;                               % Cập nhật trọng số
    P = (P - k * y * P) / lambda;                % Cập nhật ma trận P
end

% ---- Chạy RLS liên tục qua toàn bộ Payload (Decision-Directed) ----
for i = 1:500
    y_k = rx_payload(i);
    
    % 1. Lấy dữ liệu tham chiếu (Reference)
    if ismember(i, pilot_indices)
        % Nếu là Pilot: Dùng chuẩn đã biết (Data-Aided)
        d_k = pilot_sym_ideal; 
    else
        % Nếu là Data: Ra quyết định QPSK cứng dựa trên trọng số W hiện tại (Decision-Directed)
        eq_temp = W * y_k;
        d_k = sign(real(eq_temp)) + 1j * sign(imag(eq_temp)); 
    end
    
    % 2. Cân bằng tín hiệu nhận được lưu vào mảng kết quả
    rx_payload_eq(i) = W * y_k;
    
    % 3. Cập nhật RLS cho symbol tiếp theo (dùng chung cho cả Pilot và Data)
    y_mag_sq = real(y_k)^2 + imag(y_k)^2;
    k = (P * conj(y_k)) / (lambda + P * y_mag_sq);
    e = d_k - rx_payload_eq(i);
    W = W + k * e;
    P = (P - k * y_k * P) / lambda;
    
    % Lưu vết trọng số W để vẽ đồ thị
    W_track(i) = W;
end

% Rút trích chỉ lấy Data (đã loại bỏ các vị trí pilot)
rx_data_eq = rx_payload_eq(data_indices); 

% ---- (Tuỳ chọn) Kiểm tra hội tụ: vẽ |W| và pha theo symbol index ----
figure;
subplot(2,1,1);
plot(abs(W_track), 'LineWidth', 1.5); grid on;
title('Biên độ trọng số |W| (xấp xỉ 1/|H|) cập nhật liên tục qua 500 symbol');
xlabel('Symbol index'); ylabel('|W|');

subplot(2,1,2);
plot(rad2deg(angle(W_track)), 'LineWidth', 1.5); grid on;
title('Pha của W cập nhật liên tục qua 500 symbol');
xlabel('Symbol index'); ylabel('Pha (độ)');

% 6. GIẢI ĐIỀU CHẾ QPSK (DEMAPPING) VÀ SO SÁNH
% Giải mã: Phần thực âm -> bit 1, Phần ảo âm -> bit 1
rx_bits = zeros(length(rx_data_eq)*2, 1);
rx_bits(1:2:end) = real(rx_data_eq) < 0;
rx_bits(2:2:end) = imag(rx_data_eq) < 0;

% Tái tạo lại data_bits ở máy phát để tính tỉ lệ lỗi (BER)
rng(123); % Cùng seed
tx_data_bits_ideal = randi([0 1], 400 * 2, 1);

% Tính Bit Error Rate
num_errors = sum(rx_bits ~= tx_data_bits_ideal);
BER = num_errors / length(tx_data_bits_ideal);

% 7. HIỂN THỊ KẾT QUẢ
disp('--- KẾT QUẢ TRUYỀN NHẬN (dùng RLS channel estimation) ---');
fprintf('Số bits truyền: %d\n', length(tx_data_bits_ideal));
fprintf('Số bits lỗi: %d\n', num_errors);
fprintf('Tỉ lệ lỗi bit (BER): %f\n', BER);

figure;
scatter(real(rx_data_eq), imag(rx_data_eq), 'b.'); hold on;
scatter([1 -1 1 -1], [1 1 -1 -1], 'rx', 'LineWidth', 2);
title('Chòm sao tín hiệu nhận được (Đã cân bằng kênh bằng RLS)');
xlabel('In-phase'); ylabel('Quadrature');
grid on; axis square;

%% ==================== HÀM RLS ====================
function [nfData,newWeight,newInvConv] = rls_engine(fData,refData,oldWeight,oldInvConv,lambda)
    scaledSig = oldWeight.*fData;
    errsig = refData - scaledSig;
    % RLS algorithm
    XP = conj(fData).*oldInvConv;
    invDen = 1./(lambda + XP.*fData);
    K = invDen.*(oldInvConv.*fData);
    newInvConv = (1/lambda)*(oldInvConv - K.*XP);
    newWeight = oldWeight + errsig .* conj(K);
    nfData = fData.*oldWeight;
end
