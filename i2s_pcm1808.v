module i2s_pcm1808_master #(
    parameter FRAME_WIDTH = 32,
    parameter DATA_WIDTH  = 24
) (
    // INPUT
    input  wire clk,      // 24.576MHz
    input  wire reset,
    input  wire data_in,
    input  wire start,
    // OUTPUT
    output reg  bclk,     // 6.144MHz
    output reg  lrclk,    // 96KHz
    output reg  [DATA_WIDTH-1:0] data_o
);

    // STATE MACHINE
    localparam IDLE = 2'b00;
    localparam READ = 2'b01;
    localparam WAIT = 2'b10;
    reg [1:0] state_reg, state_next;

    // COUNTER
    // Sửa thành 6 bit (chứa được giá trị 32). 5 bit chỉ đếm được đến 31.
    reg [5:0] bit_cnt_reg, bit_cnt_next; 
    reg [1:0] bclk_cnt_reg, bclk_cnt_next;

    // REGISTER
    reg [DATA_WIDTH - 1 : 0] shift_reg, shift_reg_next;
    reg [DATA_WIDTH - 1 : 0] data_o_next; 
    
    // CLOCK & FLAGS
    reg bclk_next, lrclk_next;
    reg rx_done_reg, rx_done_next;
    reg busy_reg, busy_next;


    //================================================
    // KHỐI TUẦN TỰ (SEQUENTIAL)
    //================================================
    always @(negedge clk) begin
        if (reset) begin
            state_reg    <= IDLE;           // Thêm reset cho FSM
            bit_cnt_reg  <= 6'b0; 
            bclk_cnt_reg <= 2'b0;
            shift_reg    <= 24'b0;
            data_o       <= 24'b0;
            bclk         <= 1'b0;
            lrclk        <= 1'b0;
            rx_done_reg  <= 1'b0;
            busy_reg     <= 1'b0;
        end else begin
            state_reg    <= state_next;
            bit_cnt_reg  <= bit_cnt_next; 
            bclk_cnt_reg <= bclk_cnt_next;
            shift_reg    <= shift_reg_next;
            data_o       <= data_o_next;
            bclk         <= bclk_next;
            lrclk        <= lrclk_next;
            rx_done_reg  <= rx_done_next;
            busy_reg     <= busy_next;
        end
    end

    //================================================
    // KHỐI TỔ HỢP (COMBINATIONAL)
    //================================================
    always @(*) begin
        // 1. Khởi tạo giá trị mặc định để tránh chốt (latch)
        state_next     = state_reg;
        bit_cnt_next   = bit_cnt_reg; 
        bclk_cnt_next  = bclk_cnt_reg;
        shift_reg_next = shift_reg;
        data_o_next    = data_o;
        bclk_next      = bclk;
        lrclk_next     = lrclk;
        rx_done_next   = 1'b0;
        busy_next      = busy_reg;

        // 2. FSM Logic
        case (state_reg)
            IDLE: begin
                busy_next = 1'b0;
                bclk_cnt_next = 2'b0;
                bit_cnt_next = 6'b0;
                if (start) begin
                    state_next = READ;
                    busy_next = 1'b1;
                end
            end

            READ : begin  
                lrclk_next = 1'b0;    // Kênh trái
                bclk_cnt_next = bclk_cnt_reg + 1'b1;
                
                // Tạo duty cycle 50% cho BCLK
                if (bclk_cnt_reg == 2'b00) begin
                    bclk_next = 1'b0; // Cạnh xuống (PCM1808 xuất data)
                end
                else if (bclk_cnt_reg == 2'b10) begin  
                    bclk_next = 1'b1; // Cạnh lên (FPGA đọc data)
                    
                    // Lấy dữ liệu, bỏ qua bit đầu tiên theo chuẩn I2S
                    if (bit_cnt_reg > 0 && bit_cnt_reg <= DATA_WIDTH) begin  
                        shift_reg_next = {shift_reg[DATA_WIDTH - 2 : 0], data_in};
                    end
                    bit_cnt_next = bit_cnt_reg + 1'b1;
                end
                else if (bclk_cnt_reg == 2'b11) begin
                    if (bit_cnt_reg == FRAME_WIDTH) begin
                        data_o_next = shift_reg;
                        bit_cnt_next = 6'b0;
                        state_next = WAIT;
                        rx_done_next = 1'b1;
                    end
                end              
            end

            WAIT : begin  
                lrclk_next = 1'b1;   // Kênh phải
                bclk_cnt_next = bclk_cnt_reg + 1'b1;
                
                if (bclk_cnt_reg == 2'b00) begin
                    bclk_next = 1'b0;
                end
                else if (bclk_cnt_reg == 2'b10) begin
                    bclk_next = 1'b1;
                    bit_cnt_next = bit_cnt_reg + 1'b1;
                end
                else if (bclk_cnt_reg == 2'b11) begin
                    if (bit_cnt_reg == FRAME_WIDTH) begin
                        bit_cnt_next = 6'b0;
                        // Kiểm tra start để lặp lại liên tục hoặc dừng
                        if (start) state_next = READ;
                        else       state_next = IDLE;
                    end
                end              
            end

            default: begin
                state_next = IDLE;
            end
        endcase
    end

endmodule
