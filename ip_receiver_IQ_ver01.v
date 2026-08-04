module iq_axis_receiver #(
    parameter WIDTH = 16
)(
    input  wire                     clk,
    input  wire                     rst_n,

    //================ AXI4-Stream Slave =================
    input  wire [31:0]              s_axis_tdata,
    input  wire                     s_axis_tvalid,
    output wire                     s_axis_tready,

    //================ AXI4-Stream Master (I) ============
    output reg  [WIDTH-1:0]         m_axis_i_tdata,
    output reg                      m_axis_i_tvalid,
    input  wire                     m_axis_i_tready,

    //================ AXI4-Stream Master (Q) ============
    output reg  [WIDTH-1:0]         m_axis_q_tdata,
    output reg                      m_axis_q_tvalid,
    input  wire                     m_axis_q_tready
);

    // Ch? nh?n m?u m?i khi c? hai FIR ??u s?n sàng
    assign s_axis_tready = (~m_axis_i_tvalid || m_axis_i_tready) &&
                           (~m_axis_q_tvalid || m_axis_q_tready);

    always @(posedge clk or negedge rst_n)
    begin
        if(!rst_n)
        begin
            m_axis_i_tdata  <= 32'sd0;
            m_axis_q_tdata  <= 32'sd0;
            m_axis_i_tvalid <= 1'b0;
            m_axis_q_tvalid <= 1'b0;
        end
        else
        begin
            // Khi FIR nh?n xong thì h? valid
            if(m_axis_i_tvalid && m_axis_i_tready)
                m_axis_i_tvalid <= 1'b0;

            if(m_axis_q_tvalid && m_axis_q_tready)
                m_axis_q_tvalid <= 1'b0;

            // Nh?n m?t m?u m?i t? DMA/FIFO
            if(s_axis_tvalid && s_axis_tready)
            begin
                m_axis_i_tdata  <= s_axis_tdata[31:16];
                m_axis_q_tdata  <= s_axis_tdata[15:0];

                m_axis_i_tvalid <= 1'b1;
                m_axis_q_tvalid <= 1'b1;
            end
        end
    end

endmodule
