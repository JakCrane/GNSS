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

    mcu_spi spi (
        .clk      (clk),
        .mcu_sck  (mcu_sck),
        .mcu_cs   (mcu_cs),
        .mcu_mosi (mcu_mosi),
        .mcu_miso (mcu_miso),
        .rf_ld    (rf_ld)
    );

endmodule

//  todo 
//      make spi driver
//          ill give it an id to be read back
//      bring up the max
//      capture raw samples from I and Q and hold them in fpga ram, then get them onto a file to analyse them
//      prove that a python script can find the correlations
//      tracking, navigation data, position fix