module top (
    input  logic clk,
    output logic fpga_irq,
    output logic rf_shdn_n,
    output logic rf_idle_n
);
    // Need to pull these high for the chip to work which supplies the TCXO -> clk
    assign rf_shdn_n = 1'b1;
    assign rf_idle_n = 1'b1;

    logic [22:0] counter = '0;
    logic        irq_q = 1'b0;

    assign fpga_irq = irq_q;

    always_ff @(posedge clk) begin
        if (counter == 23'd8_183_999) begin
            counter <= '0;
            irq_q <= ~irq_q;
        end else begin
            counter <= counter + 1'b1;
        end
    end

endmodule