module regs #(
    parameter int NCH = 4                       // tracking channels (1..8)
) (
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

    input  logic       active,                  // from mcu_spi: CS low

    // per-channel signals, channel i in bits [i*W +: W]
    output logic [NCH-1:0]    ch_reset,
    output logic [NCH*6-1:0]  ch_prn,
    output logic [NCH*32-1:0] ch_code_fword,
    output logic [NCH*32-1:0] ch_carr_fword,
    output logic [NCH*16-1:0] ch_slew,
    output logic [NCH-1:0]    ch_slew_load,
    input  logic [NCH-1:0]    dump,
    input  logic [NCH*20-1:0] ie, qe, ip, qp, il, ql
);

    localparam logic [6:0] A_ID        = 7'h00;
    localparam logic [6:0] A_STATUS    = 7'h01;
    localparam logic [6:0] A_CTRL      = 7'h02;   // write bit 0 = 1 → start capture
    localparam logic [6:0] A_CAPDATA   = 7'h03;   // each read = next captured byte
    localparam logic [6:0] A_DUMPFLAGS = 7'h04;   // bit i = channel i dumped since last read; reading clears
    localparam logic [6:0] A_NCH       = 7'h05;   // number of channels in this build
    localparam logic [6:0] A_CH_SEL    = 7'h0F;   // which channel 0x10..0x32 refer to
    localparam logic [6:0] A_CH_CTRL   = 7'h10;   // bit 0 = hold channel in reset
    localparam logic [6:0] A_CH_PRN    = 7'h11;
    localparam logic [6:0] A_CODE      = 7'h12;   // 4 bytes LE: 0x12..0x15
    localparam logic [6:0] A_CARR      = 7'h16;   // 4 bytes LE: 0x16..0x19
    localparam logic [6:0] A_SLEW      = 7'h1A;   // 2 bytes LE: 0x1A..0x1B, high byte arms it
    localparam logic [6:0] A_RESULTS   = 7'h20;   // 6 x int24 LE + dump count: 0x20..0x32
    localparam logic [6:0] A_DUMPCNT   = 7'h32;

    // ---------------------------------------------------------------- SPI framing
    logic [6:0] addr = '0;
    wire        hdr_write = rx_byte[7];
    wire  [6:0] hdr_addr  = rx_byte[6:0];
    logic       is_write  = 1'b0;            // remembered from the header

    wire [6:0] next_addr = (addr == A_CAPDATA) ? addr : addr + 1'b1;
    wire [6:0] rd_addr   = rx_first ? hdr_addr : next_addr;

    wire wr = rx_valid && !rx_first && is_write;

    wire  reading = rx_first ? !hdr_write : !is_write;
    logic loaded_capdata = 1'b0;

    logic active_d = 1'b0;
    always_ff @(posedge clk) active_d <= active;
    wire frame_end = active_d && !active;

    always_ff @(posedge clk) begin
        if (rx_first) begin
            addr     <= hdr_addr;            // byte 1 belongs to the header's address
            is_write <= hdr_write;
        end else if (rx_valid) begin
            addr <= next_addr;             // move to the next slot
        end
        if (rx_valid) loaded_capdata <= reading && (rd_addr == A_CAPDATA);
    end

    // ---------------------------------------------------------------- channel select
    // Writes to 0x10..0x1B and reads of 0x20..0x32 go to channel `sel`. A write frame
    // starting at 0x0F selects and configures in one go: [sel, ctrl, prn, code, carr, slew].
    logic [2:0] sel = '0;
    wire        sel_ok = (32'(sel) < NCH);

    // ---------------------------------------------------------------- per-channel shadows
    // Written byte by byte during a frame; copied to the live registers at frame end so a
    // channel never sees half a frequency word.
    logic [NCH-1:0]    sh_rst  = '1;                       // start held in reset
    logic [NCH*6-1:0]  sh_prn  = '0;
    logic [NCH*32-1:0] sh_code = {NCH{32'h1000_0000}};     // nominal 1.023 MHz
    logic [NCH*32-1:0] sh_carr = {NCH{32'h4000_0000}};     // nominal 4.092 MHz
    logic [NCH*16-1:0] sh_slew = '0;

    always_ff @(posedge clk) begin
        if (wr) begin
            if (addr == A_CH_SEL) sel <= rx_byte[2:0];
            if (sel_ok) begin
                case (addr)
                    A_CH_CTRL:     sh_rst[sel]                   <= rx_byte[0];
                    A_CH_PRN:      sh_prn [sel*6 +: 6]           <= rx_byte[5:0];
                    A_CODE + 7'd0: sh_code[sel*32 +  0 +: 8]     <= rx_byte;
                    A_CODE + 7'd1: sh_code[sel*32 +  8 +: 8]     <= rx_byte;
                    A_CODE + 7'd2: sh_code[sel*32 + 16 +: 8]     <= rx_byte;
                    A_CODE + 7'd3: sh_code[sel*32 + 24 +: 8]     <= rx_byte;
                    A_CARR + 7'd0: sh_carr[sel*32 +  0 +: 8]     <= rx_byte;
                    A_CARR + 7'd1: sh_carr[sel*32 +  8 +: 8]     <= rx_byte;
                    A_CARR + 7'd2: sh_carr[sel*32 + 16 +: 8]     <= rx_byte;
                    A_CARR + 7'd3: sh_carr[sel*32 + 24 +: 8]     <= rx_byte;
                    A_SLEW + 7'd0: sh_slew[sel*16 +  0 +: 8]     <= rx_byte;
                    A_SLEW + 7'd1: sh_slew[sel*16 +  8 +: 8]     <= rx_byte;
                    default: ;
                endcase
            end
        end
    end

    // slew: writing a channel's high byte arms it; it fires once, at frame end
    logic [NCH-1:0] slew_pend   = '0;
    logic [NCH-1:0] slew_load_r = '0;
    always_ff @(posedge clk) begin
        slew_load_r <= '0;
        if (frame_end) begin
            slew_load_r <= slew_pend;
            slew_pend   <= '0;
        end else if (wr && sel_ok && addr == A_SLEW + 7'd1) begin
            slew_pend[sel] <= 1'b1;
        end
    end

    // live registers: all channels copied at every frame end (unwritten ones don't change)
    logic [NCH-1:0]    rst_r  = '1;
    logic [NCH*6-1:0]  prn_r  = '0;
    logic [NCH*32-1:0] code_r = {NCH{32'h1000_0000}};
    logic [NCH*32-1:0] carr_r = {NCH{32'h4000_0000}};
    always_ff @(posedge clk) begin
        if (frame_end) begin
            rst_r  <= sh_rst;
            prn_r  <= sh_prn;
            code_r <= sh_code;
            carr_r <= sh_carr;
        end
    end

    assign ch_reset      = rst_r;
    assign ch_prn        = prn_r;
    assign ch_code_fword = code_r;
    assign ch_carr_fword = carr_r;
    assign ch_slew       = sh_slew;
    assign ch_slew_load  = slew_load_r;

    // ---------------------------------------------------------------- dumps
    logic [NCH*8-1:0] dump_cnt = '0;
    for (genvar i = 0; i < NCH; i++) begin : g_cnt
        always_ff @(posedge clk) if (dump[i]) dump_cnt[i*8 +: 8] <= dump_cnt[i*8 +: 8] + 1'b1;
    end

    // Sticky "new dump" flags, one per channel. Reading A_DUMPFLAGS returns them and
    // clears exactly what was returned; a dump on the same clock is kept for next time.
    logic [NCH-1:0] dump_flags = '0;
    wire flags_read = rx_first && !hdr_write && (hdr_addr == A_DUMPFLAGS);
    always_ff @(posedge clk)
        dump_flags <= (flags_read ? '0 : dump_flags) | dump;

    // ---------------------------------------------------------------- results (selected channel)
    // the selected channel's correlators, sign-extended 20 -> 24 bits
    wire [19:0] s_ie = ie[sel*20 +: 20], s_qe = qe[sel*20 +: 20];
    wire [19:0] s_ip = ip[sel*20 +: 20], s_qp = qp[sel*20 +: 20];
    wire [19:0] s_il = il[sel*20 +: 20], s_ql = ql[sel*20 +: 20];
    wire [151:0] results_live = sel_ok ? {dump_cnt[sel*8 +: 8],
                                          {{4{s_ql[19]}}, s_ql}, {{4{s_il[19]}}, s_il},
                                          {{4{s_qp[19]}}, s_qp}, {{4{s_ip[19]}}, s_ip},
                                          {{4{s_qe[19]}}, s_qe}, {{4{s_ie[19]}}, s_ie}}
                                       : '0;

    logic [151:0] snap = '0;
    always_ff @(posedge clk)
        if (rx_first) snap <= results_live;       // freeze everything when a frame starts

    wire [151:0] results = rx_first ? results_live : snap;
    wire [4:0]   res_idx = rd_addr - A_RESULTS;

    // ---------------------------------------------------------------- read mux
    always_comb begin
        if (rd_addr >= A_RESULTS && rd_addr <= A_DUMPCNT) begin
            tx_byte = results[res_idx*8 +: 8];               // byte number res_idx
        end else begin
            case (rd_addr)
                A_ID:        tx_byte = 8'h5A;
                A_STATUS:    tx_byte = {6'b0, cap_busy, ld};
                A_CAPDATA:   tx_byte = cap_rdata;
                A_DUMPFLAGS: tx_byte = 8'(dump_flags);
                A_NCH:       tx_byte = 8'(NCH);
                A_CH_SEL:    tx_byte = {5'b0, sel};
                default:     tx_byte = 8'h00;
            endcase
        end
    end

    assign cap_start = wr && (addr == A_CTRL) && rx_byte[0];
    assign cap_next  = tx_taken && loaded_capdata;

endmodule