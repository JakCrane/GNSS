module top (
    input  logic clk,

    input logic mcu_sck, mcu_cs, mcu_mosi,
    output logic mcu_miso,
    
    // output logic mcu_irq,
    output logic rf_shdn_n,
    output logic rf_idle_n,
    input logic rf_ld

);
    // Need to pull these high for the chip to work which supplies the TCXO -> clk
    assign rf_shdn_n = 1'b1;
    assign rf_idle_n = 1'b1;

    logic [7:0] cmd, reply;
    logic       cmd_valid;
    logic       ld_sync;

    sync2 ld_sync_i (
        .clk(clk), 
        .raw_data(rf_ld),
        .synchronised_data(ld_sync)
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
            default: reply = 8'h00;
        endcase
    end

endmodule

//  todo 
//      make spi driver
//          ill give it an id to be read back
//      bring up the max
//      capture raw samples from I and Q and hold them in fpga ram, then get them onto a file to analyse them
//      prove that a python script can find the correlations
//      tracking, navigation data, position fix