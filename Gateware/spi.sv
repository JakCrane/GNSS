module mcu_spi (
    input logic clk,
    input logic mcu_sck, mcu_cs, mcu_mosi,
    output logic mcu_miso
);

    logic [1:0] cs_s   = '1; // read multiple times and ignore the first one to ensure youre reading the settled value and idles high
    logic [1:0] mosi_s = '0; // _s for synchronised and idles low
    logic [2:0] sck_s  = '0; // _s for synchronised and idles low

    logic [5:0] bit_cnt = '0; // where you are in the frame
    logic [7:0] cmd     = '0;
    logic [7:0] sh_out  = '0;

    wire active = !cs_s[1]; // active when cs low
    wire sck_rise = (sck_s[2:1] == 2'b01);
    wire sck_fall = (sck_s[2:1] == 2'b10);

    wire [7:0] cmd_next = {cmd[6:0], mosi_s[1]};
    
    assign mcu_miso = sh_out[7];

    always_ff @(posedge clk) begin // if it was clocked on mcu_sck then you couldnt perfrom the active check
        if (!active) begin
            bit_cnt <= '0;
            cmd <= 0;
            sh_out <= '0;
        end else if (sck_rise) begin
            if (bit_cnt < 8) begin
                cmd <= {cmd[6:0], mosi_s[1]}; // MSB-first        
            end

            if (bit_cnt == 7) begin
                case (cmd_next)
                    8'h01:   sh_out <= 8'h5A;   // read ID
                    default: sh_out <= 8'h00;   // unknown command
                endcase
            end

            bit_cnt <= bit_cnt + 1'b1;
        
        end else if (sck_fall && bit_cnt > 8) begin
            sh_out <= {sh_out[6:0], 1'b0};

        end
    end

    always_ff @(posedge clk) begin
        cs_s   <= {cs_s[0],    mcu_cs};
        mosi_s <= {mosi_s[0],  mcu_mosi};
        sck_s  <= {sck_s[1:0], mcu_sck};
    end
endmodule 

// note that this drives miso always (not deep for this specific use but watch out for if multiple slaves on bus)