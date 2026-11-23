module ones_counter (
    input  logic       clk,
    input  logic       bit_in,      // already synchronised
    output logic [7:0] ratio
);
    logic [14:0] win_cnt      = '0;   // position in window, 0..32767
    logic [15:0] ones         = '0;   // running count for this window
    logic [15:0] ones_latched = '0;   // result of the last complete window

    always_ff @(posedge clk) begin
        win_cnt <= win_cnt + 1'b1;               // wraps 32767 → 0 by itself
        if (win_cnt == '1) begin                 // last sample of the window
            ones_latched <= ones + bit_in;       // save finished count
            ones         <= '0;                  // start next window
        end else begin
            ones <= ones + bit_in;
        end
    end

    assign ratio = ones_latched[15:8];
endmodule