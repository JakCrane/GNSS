module top (
    input  logic clk,

    input logic mcu_sck, mcu_cs, mcu_mosi,
    output logic mcu_miso,
    
    // output logic mcu_irq,
    output logic rf_shdn_n,
    output logic rf_idle_n,
    input logic rf_ld,
    input logic rf_i0,
    input logic rf_i1

);
    // Need to pull these high for the chip to work which supplies the TCXO -> clk
    assign rf_shdn_n = 1'b1;
    assign rf_idle_n = 1'b1;

    logic ld_sync;
    logic i0_sync;
    logic i1_sync;

    logic busy;
    logic [7:0] rdata;

    sync2  #(.INIT(1'b0)) ld_sync_i ( // parameter redundant for now but for readability
        .clk(clk), 
        .raw_data(rf_ld),
        .synchronised_data(ld_sync)
    );

    sync2 rf_i0_i (
        .clk(clk), 
        .raw_data(rf_i0),
        .synchronised_data(i0_sync)
    );

    sync2 rf_i1_i (
        .clk(clk), 
        .raw_data(rf_i1),
        .synchronised_data(i1_sync)
    );

    logic       spi_active, rx_valid, rx_first, tx_taken;
    logic [7:0] rx_byte, tx_byte;

    mcu_spi spi (
        .clk       (clk),
        .mcu_sck   (mcu_sck),
        .mcu_cs    (mcu_cs),
        .mcu_mosi  (mcu_mosi),
        .tx_byte   (tx_byte),
        .mcu_miso  (mcu_miso),
        .active    (spi_active),
        .rx_byte   (rx_byte),
        .rx_valid  (rx_valid),
        .rx_first  (rx_first),
        .tx_taken  (tx_taken)
    );

    localparam int NCH = 4;                     // tracking channels

    wire signed [2:0] sample = i1_sync ? (i0_sync ?  3'sd3 :  3'sd1)
                                   : (i0_sync ? -3'sd3 : -3'sd1);

    // per-channel buses: channel i in bits [i*W +: W]
    logic [NCH-1:0]    ch_reset, ch_slew_load, dump;
    logic [NCH*6-1:0]  ch_prn;
    logic [NCH*32-1:0] ch_code_fword, ch_carr_fword;
    logic [NCH*16-1:0] ch_slew;
    logic [NCH*20-1:0] ie, qe, ip, qp, il, ql;

    for (genvar i = 0; i < NCH; i++) begin : g_ch
        channel ch (
            .clk           (clk),
            .reset         (ch_reset[i]),
            .prn           (ch_prn[i*6 +: 6]),
            .code_fword    (ch_code_fword[i*32 +: 32]),
            .carrier_fword (ch_carr_fword[i*32 +: 32]),
            .sample        (sample),
            .ie (ie[i*20 +: 20]), .qe (qe[i*20 +: 20]),
            .ip (ip[i*20 +: 20]), .qp (qp[i*20 +: 20]),
            .il (il[i*20 +: 20]), .ql (ql[i*20 +: 20]),
            .dump          (dump[i]),
            .early(), .prompt(), .late(), .epoch(),
            .slew          (ch_slew[i*16 +: 16]),
            .slew_load     (ch_slew_load[i])
        );
    end

    logic cap_start, cap_next;

    regs #(.NCH(NCH)) regs_i (
        .clk            (clk),
        .rx_byte        (rx_byte),
        .rx_valid       (rx_valid),
        .rx_first       (rx_first),
        .tx_byte        (tx_byte),
        .ld             (ld_sync),
        .cap_busy       (busy),
        .tx_taken       (tx_taken),
        .cap_rdata      (rdata),
        .cap_start      (cap_start),
        .cap_next       (cap_next),
        .active         (spi_active),
        .ch_reset       (ch_reset),
        .ch_prn         (ch_prn),
        .ch_code_fword  (ch_code_fword),
        .ch_carr_fword  (ch_carr_fword),
        .ch_slew        (ch_slew),
        .ch_slew_load   (ch_slew_load),
        .dump           (dump),
        .ie(ie), .qe(qe), .ip(ip), .qp(qp), .il(il), .ql(ql)
    );

    capture cap (
        .clk        (clk),
        .sample     ({i0_sync, i1_sync}), // mag, sign
        .start      (cap_start),
        .ready_next (cap_next),
        .busy       (busy),
        .rdata      (rdata)
    );


endmodule

//  todo 
//      C/A genertor
//      code nco
//      carrier nco and mixer
//      correlators