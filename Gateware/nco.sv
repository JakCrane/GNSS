module code_nco (
    input  logic        clk,
    input  logic        reset,       // phase back to 0
    input  logic [31:0] fword,       // chip rate: fword = f_chip / f_clk × 2^32 (large as it treats a large int like a decimal with many sig fig)
    output logic        chip_en,     // 1-clock pulse each time a chip boundary is crossed
    output logic [31:0] phase,       // where we are within the current chip (0 → 2^32)
    input  logic        enable
);
    // One whole chip = 2^32 units (needs 33 bits to hold).
    localparam logic [32:0] ONE_CHIP = 33'h1_0000_0000;

    // Reference value: 1/16 of a chip per clock (16.368 MHz / 16 = 1.023 MHz).
    localparam logic [31:0] NOMINAL_FWORD = 32'h1000_0000;

    logic [31:0] phase_now  = '0;    // where we are this clock
    logic [32:0] phase_next;         // where we'd be after this clock's step (before wrapping)
    logic        past_end_of_chip;
    
    assign phase_next     = {1'b0, phase_now} + {1'b0, fword}; // 1'b0 pads the numbers as theyre different sizes to sum
    
    // Did that step take us past the end of the chip?
    assign past_end_of_chip = (phase_next >= ONE_CHIP);              // carry out = crossed a chip boundary

    always_ff @(posedge clk) begin
        if (reset) begin
            phase_now <= '0;
        end else if (enable) begin
            if (past_end_of_chip) begin
                phase_now <= phase_next - ONE_CHIP;     // start the next chip, keep the leftover
            end else begin
                phase_now <= phase_next[31:0];          // still inside this chip
            end
        end
    end

    assign chip_en = enable && past_end_of_chip;
    assign phase   = phase_now;

endmodule
