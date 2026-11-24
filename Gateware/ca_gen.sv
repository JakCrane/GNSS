module ca_gen (
    input  logic       clk,
    input  logic       reset,     // load the start state (chip 0)
    input  logic       chip_en,   // 1-clock pulse: move on one chip
    input  logic [5:0] prn,       // 1..32
    output logic       chip,      // current chip (0/1)
    output logic       epoch      // high during chip 0 = start of the 1 ms code
);
    logic [9:0] g1 = '1;
    logic [9:0] g2 = '1;

    logic g1_next = 1'b0;
    logic g1_num_of_1s = 4'b0;
    always_ff @(posedge clk) begin
        if (reset) begin

            g1 <= '1;

        end else if (chip_en) begin            

            g1_num_of_1s = (g1[2] + g1[9]);
            if (g1_num_of_1s == 1) begin
                g1_next = 1; 
            end else begin
                g1_next = 0; 
            end
            g1 <= {g1[8:0], g1_next};

        end
    
    end

    logic g2_next = 1'b0;
    logic g2_num_of_1s = 4'b0;
    always_ff @(posedge clk) begin
        if (reset) begin

            g2 <= '1;

        end else if (chip_en) begin  

            g2_num_of_1s = (g2[1] + g2[2] + g2[5] + g2[7] + g2[8] + g2[9]);
            if (g2_num_of_1s == 1 | g2_num_of_1s == 3 | g2_num_of_1s == 5) begin
                g2_next = 1; 
            end else begin
                g2_next = 0; 
            end
            g2 <= {g2[8:0], g2_next};

        end
    end
    
    logic output_chip = 1'b0;

    always_comb begin
        
        if ( g1[9] ^ (g2[0] ^ g2[3])) begin
            output_chip = 1;
        end else begin
            output_chip = 0;
        end

    end

    assign chip = output_chip;
    assign epoch = (g1 == '1); // when and only when full loop completed g1 returns to all 1s

endmodule