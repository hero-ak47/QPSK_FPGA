function write_coe(h, filename, width)
% write_coe  Xuất hệ số FIR ra file .coe cho Xilinx FIR Compiler IP.
%
%   write_coe(h, filename, width)
%
%   h        : vector hệ số FIR (double, có thể âm/dương, không cần chuẩn hóa trước)
%   filename : tên file .coe xuất ra, ví dụ 'bandpass_fir.coe'
%   width    : số bit lượng tử hóa hệ số (Coefficient_Width), mặc định 16
%
%   Định dạng số: signed fixed-point, Radix = 10 (hệ thập phân, số nguyên).
%   Xilinx FIR Compiler khi Coefficient_Structure = Radix, Radix=10 sẽ tự
%   hiểu các số nguyên này là Qm.f theo Coefficient_Width và Fractional_Bits
%   bạn khai báo trong GUI IP (thường Fractional_Bits = width-1, tức Q1.(width-1)).

  if nargin < 3
    width = 16;
  end

  h = h(:); % ép về vector cột

  max_abs = max(abs(h));
  if max_abs == 0
    error('Tat ca he so bang 0, kiem tra lai bo loc dau vao.');
  end

  % Chuẩn hóa để hệ số lớn nhất chiếm gần hết dải biểu diễn (chừa margin 1%
  % tránh tràn số do làm tròn), rồi lượng tử hóa về số nguyên width-bit có dấu.
  margin = 0.99;
  scale = margin * (2^(width-1) - 1) / max_abs;

  h_q = round(h * scale);

  % Giới hạn cứng trong khoảng biểu diễn được của width-bit có dấu
  qmax =  2^(width-1) - 1;
  qmin = -2^(width-1);
  h_q(h_q > qmax) = qmax;
  h_q(h_q < qmin) = qmin;

  fid = fopen(filename, 'w');
  if fid == -1
    error('Khong the mo file %s de ghi.', filename);
  end

  fprintf(fid, '; He so FIR xuat tu MATLAB - %s\n', filename);
  fprintf(fid, '; So taps = %d, Coefficient_Width = %d bit, scale = %.6f\n', ...
          length(h_q), width, scale);
  fprintf(fid, 'Radix = 10;\n');
  fprintf(fid, 'Coefficient_Width = %d;\n', width);
  fprintf(fid, 'CoefData =\n');

  for i = 1:length(h_q)
    if i < length(h_q)
      fprintf(fid, '%d,\n', h_q(i));
    else
      fprintf(fid, '%d;\n', h_q(i)); % dòng cuối kết thúc bằng dấu ;
    end
  end

  fclose(fid);

  fprintf('Da xuat %s: %d taps, %d-bit, scale=%.6f (fractional bits goi y = %d)\n', ...
          filename, length(h_q), width, scale, width-1);
end
