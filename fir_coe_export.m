%% XUẤT HỆ SỐ FIR RA FILE .COE CHO IP FIR COMPILER (VIVADO)
% Dùng đúng thông số hệ thống của bộ thu/phát: Fs=48000, Fc=8000, Rs=200

Fs = 96000;
Fc = 12000;
Rb = 400;
Rs = Rb/2;      % 200 baud
L  = Fs/Rs;     % 240 samples/symbol

beta = 0.5;
span = 6;

COEF_WIDTH = 16;   % số bit lượng tử hóa hệ số, chỉnh theo cấu hình IP (thường 16 hoặc 18)

%% 1. BỘ LỌC BANDPASS (FIR thay cho IIR butter, vì FIR Compiler cần hệ số FIR)
% Băng thông cần giữ lại quanh Fc: (1+beta)*Rs/2 mỗi bên ~ 150 Hz,
% chọn biên rộng hơn 1 chút để an toàn: +-1500 Hz quanh Fc = [6500 9500] Hz
f_low  = Fc - 1000;
f_high = Fc + 1000;

N_bp = 200;  % bậc bộ lọc (order), tăng lên nếu cần dải chuyển tiếp hẹp hơn / suy hao mạnh hơn
h_bp = fir1(N_bp, [f_low f_high]/(Fs/2), 'bandpass', kaiser(N_bp+1, 6));

write_coe(h_bp, 'bandpass_fir.coe', COEF_WIDTH);

%% 2. BỘ LỌC RRC (Matched Filter / Pulse Shaping)
% Trong MATLAB thật, dùng hàm có sẵn:
h_rrc = rcosdesign(beta, span, L, 'sqrt');
% (Trong Octave test không có rcosdesign nên dùng rrc_manual thay thế tương đương;
%  khi chạy trên MATLAB thật, xóa 2 dòng dưới và chỉ giữ dòng rcosdesign ở trên.)

write_coe(h_rrc, 'rrc_fir.coe', COEF_WIDTH);

%% 3. KIỂM TRA NHANH ĐÁP ỨNG TẦN SỐ (tùy chọn, bỏ qua nếu không cần)
figure;
subplot(2,1,1);
[H_bp, f_bp] = freqz(h_bp, 1, 2048, Fs);
plot(f_bp, 20*log10(abs(H_bp))); grid on;
title('Dap ung tan so - Bandpass FIR'); xlabel('Hz'); ylabel('dB');
xlim([0 Fs/2]);

subplot(2,1,2);
[H_rrc, f_rrc] = freqz(h_rrc, 1, 2048, Fs);
plot(f_rrc, 20*log10(abs(H_rrc))); grid on;
title('Dap ung tan so - RRC'); xlabel('Hz'); ylabel('dB');
xlim([0 Fs/2]);

fprintf('\nSo tap Bandpass FIR: %d\n', length(h_bp));
fprintf('So tap RRC FIR     : %d\n', length(h_rrc));
