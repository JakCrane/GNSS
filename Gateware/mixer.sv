module carrier_mixer (
    input  logic               clk,
    input  logic               reset,          // carrier phase back to 0
    input  logic signed [2:0]  sample,         // incoming sample: ±1 / ±3
    input  logic        [31:0] carrier_fword,  // IF + Doppler; nominal 32'h4000_0000 = 4.092 MHz
    output logic signed [3:0]  i_mix,          // sample × cos
    output logic signed [3:0]  q_mix           // sample × sin
);

    logic [31:0] carr_phase;
    code_nco carr_nco (
        .clk     (clk),
        .reset   (reset),
        .fword   (carrier_fword),     // nominal 2^30 = 32'h4000_0000 → 4.092 MHz
        .chip_en (),                   // not needed: leave unconnected
        .phase   (carr_phase)
    );

    logic signed [3:0] lo_cos;
    logic signed [3:0] lo_sin;
    always_comb begin
        case (carr_phase[31:29])            // which 45° slice of the cycle we're in
            3'd0: begin lo_cos =  3'sd2; lo_sin =  3'sd1; end   //  22.5°
            3'd1: begin lo_cos =  3'sd1; lo_sin =  3'sd2; end   //  67.5°
            3'd2: begin lo_cos = -3'sd1; lo_sin =  3'sd2; end   // 112.5°
            3'd3: begin lo_cos = -3'sd2; lo_sin =  3'sd1; end   // 157.5°
            3'd4: begin lo_cos = -3'sd2; lo_sin = -3'sd1; end   // 202.5°
            3'd5: begin lo_cos = -3'sd1; lo_sin = -3'sd2; end   // 247.5°
            3'd6: begin lo_cos =  3'sd1; lo_sin = -3'sd2; end   // 292.5°
            3'd7: begin lo_cos =  3'sd2; lo_sin = -3'sd1; end   // 337.5°
            default: begin lo_cos = 3'sd0; lo_sin = 3'sd0; end
        endcase
    end

    logic signed [3:0] i_mix_r = '0;
    logic signed [3:0] q_mix_r = '0;

    always_ff @(posedge clk) begin
        i_mix_r = sample * lo_cos;
        q_mix_r = sample * lo_sin;
    end


    assign i_mix = i_mix_r;
    assign q_mix = q_mix_r;

    
endmodule