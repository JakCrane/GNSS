module regs (
    input  logic       clk,
    // from mcu_spi
    input  logic [7:0] rx_byte,
    input  logic       rx_valid,
    input  logic       rx_first,
    output logic [7:0] tx_byte,
    // things to report
    input  logic       ld,
    input  logic       cap_busy,

    input  logic       tx_taken,
    input  logic [7:0] cap_rdata,
    output logic       cap_start,
    output logic       cap_next,

    input  logic        active,          // from mcu_spi: CS low
    output logic        ch_reset,
    output logic [5:0]  ch_prn,
    output logic [31:0] ch_code_fword,
    output logic [31:0] ch_carr_fword,
    input  logic        dump,
    input  logic signed [19:0] ie, qe, ip, qp, il, ql,

    output logic [15:0] ch_slew,
    output logic        ch_slew_load
);

    localparam logic [6:0] A_ID      = 7'h00;
    localparam logic [6:0] A_STATUS  = 7'h01;
    localparam logic [6:0] A_CTRL    = 7'h02;   // write bit 0 = 1 → start capture
    localparam logic [6:0] A_CAPDATA = 7'h03;   // each read = next captured byte
    localparam logic [6:0] A_CH_CTRL = 7'h10;   // bit 0 = hold channel in reset
    localparam logic [6:0] A_CH_PRN  = 7'h11;
    localparam logic [6:0] A_CODE    = 7'h12;   // 4 bytes LE: 0x12..0x15
    localparam logic [6:0] A_CARR    = 7'h16;   // 4 bytes LE: 0x16..0x19
    localparam logic [6:0] A_SLEW    = 7'h1A;   // 2 bytes LE: 0x1A..0x1B, high byte arms it
    localparam logic [6:0] A_RESULTS = 7'h20;   // 6 x int24 LE + dump count: 0x20..0x32
    localparam logic [6:0] A_DUMPCNT = 7'h32;

    logic [7:0] dump_cnt = '0;
    logic [6:0] addr = '0;
    wire        hdr_write = rx_byte[7];
    wire  [6:0] hdr_addr  = rx_byte[6:0];
    logic       is_write  = 1'b0;            // remembered from the header

    wire [6:0] next_addr = (addr == A_CAPDATA) ? addr : addr + 1'b1;
    wire [6:0] rd_addr   = rx_first ? hdr_addr : next_addr;

    wire wr = rx_valid && !rx_first && is_write;

    wire  reading = rx_first ? !hdr_write : !is_write;
    logic loaded_capdata = 1'b0;

    wire [23:0] ie24 = {{4{ie[19]}}, ie};
    wire [23:0] qe24 = {{4{qe[19]}}, qe};
    wire [23:0] ip24 = {{4{ip[19]}}, ip};
    wire [23:0] qp24 = {{4{qp[19]}}, qp};
    wire [23:0] il24 = {{4{il[19]}}, il};
    wire [23:0] ql24 = {{4{ql[19]}}, ql};
    wire [151:0] results_live = {dump_cnt, ql24, il24, qp24, ip24, qe24, ie24};

    logic [151:0] snap = '0;
    always_ff @(posedge clk)
        if (rx_first) snap <= results_live;       // freeze everything when a frame starts

    wire [151:0] results = rx_first ? results_live : snap;   // after the snap block (line 58)
    wire [4:0]   res_idx = rd_addr - A_RESULTS;

    logic [7:0]  sh_ctrl = 8'h01;              // start held in reset
    logic [5:0]  sh_prn  = '0;
    logic [31:0] sh_code = 32'h1000_0000;      // nominal 1.023 MHz
    logic [31:0] sh_carr = 32'h4000_0000;      // nominal 4.092 MHz

    logic [15:0] sh_slew     = '0;
    logic        slew_pend   = 1'b0;
    logic        slew_load_r = 1'b0;

    logic active_d = 1'b0;
    always_ff @(posedge clk) active_d <= active;
    wire frame_end = active_d && !active;
    
    // separate block: slew_pend must only be driven from one place
    always_ff @(posedge clk) begin
        slew_load_r <= 1'b0;
        if (frame_end) begin
            slew_load_r <= slew_pend;
            slew_pend   <= 1'b0;
        end else if (wr && addr == A_SLEW + 7'd1) begin
            slew_pend <= 1'b1;
        end
    end

    always_ff @(posedge clk) begin
        if (wr) begin
            case (addr)
                A_CH_CTRL:        sh_ctrl        <= rx_byte;
                A_CH_PRN:         sh_prn         <= rx_byte[5:0];
                A_CODE + 7'd0:    sh_code[7:0]   <= rx_byte;
                A_CODE + 7'd1:    sh_code[15:8]  <= rx_byte;
                A_CODE + 7'd2:    sh_code[23:16] <= rx_byte;
                A_CODE + 7'd3:    sh_code[31:24] <= rx_byte;
                A_CARR + 7'd0:    sh_carr[7:0]   <= rx_byte;
                A_CARR + 7'd1:    sh_carr[15:8]  <= rx_byte;
                A_CARR + 7'd2:    sh_carr[23:16] <= rx_byte;
                A_CARR + 7'd3:    sh_carr[31:24] <= rx_byte;
                A_SLEW + 7'd0:    sh_slew[7:0]   <= rx_byte;
                A_SLEW + 7'd1:    sh_slew[15:8]  <= rx_byte;
                default: ;
            endcase
        end
    end



    logic        ch_reset_r = 1'b1;              // held in reset until the MCU releases it
    logic [5:0]  ch_prn_r   = '0;
    logic [31:0] ch_code_r  = 32'h1000_0000;
    logic [31:0] ch_carr_r  = 32'h4000_0000;

    always_ff @(posedge clk) begin
        if (frame_end) begin
            ch_reset_r      <= sh_ctrl[0];
            ch_prn_r        <= sh_prn;
            ch_code_r <= sh_code;
            ch_carr_r <= sh_carr;
        end
    end

    assign ch_reset      = ch_reset_r;
    assign ch_prn        = ch_prn_r;
    assign ch_code_fword = ch_code_r;
    assign ch_carr_fword = ch_carr_r;

    always_ff @(posedge clk) begin
        if (rx_first) begin
            addr     <= hdr_addr;            // byte 1 belongs to the header's address
            is_write <= hdr_write;
        end else if (rx_valid) begin
            addr <= next_addr;             // move to the next slot
        end
        if (rx_valid) loaded_capdata <= reading && (rd_addr == A_CAPDATA);
    end

    always_comb begin
        if (rd_addr >= A_RESULTS && rd_addr <= A_DUMPCNT) begin
            tx_byte = results[res_idx*8 +: 8];               // byte number res_idx
        end else begin
            case (rd_addr)
                A_ID:      tx_byte = 8'h5A;
                A_STATUS:  tx_byte = {6'b0, cap_busy, ld};
                A_CAPDATA: tx_byte = cap_rdata;
                default:   tx_byte = 8'h00;
            endcase
        end
    end

    assign cap_start = wr && (addr == A_CTRL) && rx_byte[0];
    assign cap_next = tx_taken && loaded_capdata;

    always_ff @(posedge clk) if (dump) dump_cnt <= dump_cnt + 1'b1;

    assign ch_slew      = sh_slew;
    assign ch_slew_load = slew_load_r;

endmodule