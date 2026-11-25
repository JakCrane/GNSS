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

    logic [7:0] ratio0;
    logic [7:0] ratio1;

    logic busy;
    logic [7:0] rdata;

    logic [15:0] ch_slew; logic ch_slew_load; 


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

    logic signed [19:0] ie, qe, ip, qp, il, ql;
    logic               dump;
    wire signed [2:0] sample = i1_sync ? (i0_sync ?  3'sd3 :  3'sd1)
                                   : (i0_sync ? -3'sd3 : -3'sd1);

    channel ch0 (
        .clk(clk), 
        .reset(ch_reset), 
        .prn(ch_prn),
        .code_fword(ch_code_fword), 
        .carrier_fword(ch_carr_fword), 
        .sample(sample),
        .ie(ie), .qe(qe), .ip(ip), .qp(qp), .il(il), .ql(ql), 
        .dump(dump),
        .early(), 
        .prompt(), 
        .late(), 
        .epoch(),
        .slew(ch_slew),
        .slew_load(ch_slew_load)
    );

    logic cap_start, cap_next;
    logic        ch_reset;
    logic [5:0]  ch_prn;
    logic [31:0] ch_code_fword, ch_carr_fword;

    regs regs_i (
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
        .dump           (dump),
        .ie(ie), .qe(qe), .ip(ip), .qp(qp), .il(il), .ql(ql),
        .ch_slew        (ch_slew),
        .ch_slew_load   (ch_slew_load)
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