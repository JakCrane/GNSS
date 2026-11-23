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

    logic [7:0] cmd, reply;
    logic       cmd_valid;

    logic ld_sync;
    logic i0_sync;
    logic i1_sync;

    logic [7:0] ratio0;
    logic [7:0] ratio1;

    sync2 ld_sync_i (
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

    mcu_spi spi (
        .clk       (clk),
        .mcu_sck   (mcu_sck),
        .mcu_cs    (mcu_cs),
        .mcu_mosi  (mcu_mosi),
        .mcu_miso  (mcu_miso),
        .cmd       (cmd),
        .cmd_valid (cmd_valid),
        .reply     (reply)
    );

    always_comb begin
        case (cmd)
            8'h01:   reply = 8'h5A;               // ID
            8'h02:   reply = {7'b0, ld_sync};     // PLL lock
            8'h03:   reply = ratio0;              // number of 1s in mag
            8'h04:   reply = ratio1;              // number of 1s in dir
            default: reply = 8'h00;
        endcase
    end

    ones_counter counter0 (
        .clk    (clk),
        .bit_in (i0_sync),
        .ratio  (ratio0)
    );

    ones_counter counter1 (
        .clk    (clk),
        .bit_in (i1_sync),
        .ratio  (ratio1)
    );


endmodule

//  todo 
//      make spi driver
//          ill give it an id to be read back
//      bring up the max
//      capture raw samples from I and Q and hold them in fpga ram, then get them onto a file to analyse them
//      prove that a python script can find the correlations
//      tracking, navigation data, position fix