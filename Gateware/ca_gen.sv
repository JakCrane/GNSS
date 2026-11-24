module ca_gen (
    input  logic       clk,
    input  logic       reset,     // load the start state (chip 0)
    input  logic       chip_en,   // 1-clock pulse: move on one chip
    input  logic [5:0] prn,       // 1..32
    output logic       chip,      // current chip (0/1)
    output logic       epoch      // high during chip 0 = start of the 1 ms code
);

    logic [3:0] s1, s2;

    always_comb begin
        case (prn)
            6'd1:  begin s1 = 2; s2 = 6;  end
            6'd2:  begin s1 = 3; s2 = 7;  end
            6'd3:  begin s1 = 4; s2 = 8;  end
            6'd4:  begin s1 = 5; s2 = 9;  end
            6'd5:  begin s1 = 1; s2 = 9;  end
            6'd6:  begin s1 = 2; s2 = 10; end
            6'd7:  begin s1 = 1; s2 = 8;  end
            6'd8:  begin s1 = 2; s2 = 9;  end
            6'd9:  begin s1 = 3; s2 = 10; end
            6'd10: begin s1 = 2; s2 = 3;  end
            6'd11: begin s1 = 3; s2 = 4;  end
            6'd12: begin s1 = 5; s2 = 6;  end
            6'd13: begin s1 = 6; s2 = 7;  end
            6'd14: begin s1 = 7; s2 = 8;  end
            6'd15: begin s1 = 8; s2 = 9;  end
            6'd16: begin s1 = 9; s2 = 10; end
            6'd17: begin s1 = 1; s2 = 4;  end
            6'd18: begin s1 = 2; s2 = 5;  end
            6'd19: begin s1 = 3; s2 = 6;  end
            6'd20: begin s1 = 4; s2 = 7;  end
            6'd21: begin s1 = 5; s2 = 8;  end
            6'd22: begin s1 = 6; s2 = 9;  end
            6'd23: begin s1 = 1; s2 = 3;  end
            6'd24: begin s1 = 4; s2 = 6;  end
            6'd25: begin s1 = 5; s2 = 7;  end
            6'd26: begin s1 = 6; s2 = 8;  end
            6'd27: begin s1 = 7; s2 = 9;  end
            6'd28: begin s1 = 8; s2 = 10; end
            6'd29: begin s1 = 1; s2 = 6;  end
            6'd30: begin s1 = 2; s2 = 7;  end
            6'd31: begin s1 = 3; s2 = 8;  end
            6'd32: begin s1 = 4; s2 = 9;  end
            default: begin s1 = 1; s2 = 1; end
        endcase
    end




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
        
        if ( g1[9] ^ (g2[s1 - 1] ^ g2[s2 - 1])) begin
            output_chip = 1;
        end else begin
            output_chip = 0;
        end

    end

    assign chip = output_chip;
    assign epoch = (g1 == '1); // when and only when full loop completed g1 returns to all 1s

endmodule