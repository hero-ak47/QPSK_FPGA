%% THÔNG SỐ HỆ THỐNG
Fs = 96000;      % Tần số lấy mẫu (Hz)
Fc = 10000;      % Tần số sóng mang (Hz)
Rb = 100;        % Tốc độ bit (bps)
Rs = Rb/2;       % Tốc độ symbol cho QPSK (50 baud)
L  = Fs/Rs;      % Số mẫu trên một symbol (1920 samples/symbol)

Nzc = 63;
u   = 25;
Lsym = Nzc * L;              % số mẫu của MỘT khối preamble ZC sau pulse shaping
n   = (0:Nzc-1).';
pss = exp(-1j*pi*u*n.*(n+1)/Nzc);

%% 1. TẠO CẤU TRÚC KHUNG TÍN HIỆU 

% Tạo symbols Preamble 
preamble_sym = pss;

% Tạo Pilot (Cố định bit 00 -> symbol 1 + 1j)
pilot_sym = 1 + 1j;

% Tạo Dữ liệu ngẫu nhiên (450 symbols = 900 bits)
rng(123);   % Seed cố định để RX có thể tái tạo lại data_bits khi tính BER
num_data_sym = 450;
data_bits = randi([0 1], num_data_sym * 2, 1);
data_sym = (1 - 2*data_bits(1:2:end)) + 1j*(1 - 2*data_bits(2:2:end));

% Lắp ráp khung truyền (1 Pilot kèm 9 Data)
payload_sym = zeros(500, 1);
data_idx = 1;
for k = 1:50
    idx = (k-1)*10 + 1;
    payload_sym(idx) = pilot_sym;                       % Chèn Pilot
    payload_sym(idx+1 : idx+9) = data_sym(data_idx : data_idx+8); % Chèn 9 Data
    data_idx = data_idx + 9;
end

% Tổng hợp toàn bộ symbols của khung
guard = zeros(14,1);
tx_symbols = [preamble_sym; guard; payload_sym];

%% 2. TẠO DÁNG XUNG VÀ ĐIỀU CHẾ LÊN BĂNG TẦN CƠ SỞ (BASEBAND)
% Sử dụng bộ lọc Root Raised Cosine (RRC) để giới hạn băng thông, tránh ISI
beta = 0.5; % Hệ số Roll-off
span = 6;   % Độ dài bộ lọc (số symbols)
h_rrc = rcosdesign(beta, span, L, 'sqrt');

% Upsample và Lọc (Pulse Shaping)
tx_baseband = upfirdn(tx_symbols, h_rrc, L);

%% 3. ĐIỀU CHẾ SÓNG MANG (PASSBAND)
t = (0:length(tx_baseband)-1)' / Fs;
tx_passband = real(tx_baseband .* exp(1j * 2 * pi * Fc * t));

% Chuẩn hóa biên độ tín hiệu để không bị vỡ tiếng (clipping) trên loa
tx_passband = tx_passband / max(abs(tx_passband)) * 0.8;

% Thêm một chút khoảng lặng ở đầu và cuối (0.5 giây)
silence = zeros(Fs * 0.5, 1);
tx_signal_final = [silence; tx_passband; silence];

%% 4. PHÁT TÍN HIỆU RA LOA
disp('Đang phát tín hiệu ra loa...');
sound(tx_signal_final, Fs);
disp('Đã phát xong!');
%plot(tx_signal_final);
%audiowrite('tx_signal_1.wav', tx_signal_final, Fs);
%disp('Đã phát tín hiệu và lưu vào tx_signal_1.wav');
