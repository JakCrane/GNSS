module mcu_spi (
    input  logic       clk,
    input  logic       mcu_sck, mcu_cs, mcu_mosi,
    input  logic [7:0] tx_byte,     // what to send in the NEXT byte slot
    
    output logic       mcu_miso,
    output logic       active,      // CS is low (a frame is in progress)
    output logic [7:0] rx_byte,     // the byte just received
    output logic       rx_valid,    // 1-clock pulse each time a byte completes
    output logic       rx_first,    // high with rx_valid if this was byte 0 (the header)
    output logic       tx_taken     // 1-clock pulse: the byte loaded from tx_byte has started 
);

    logic cs_sync;
    sync2 #(.INIT(1'b1)) cs (
        .clk(clk), 
        .raw_data(mcu_cs),
        .synchronised_data(cs_sync)
    );

    logic mosi_sync;
    sync2 mosi (
        .clk(clk), 
        .raw_data(mcu_mosi),
        .synchronised_data(mosi_sync)
    );

    logic [2:0] sck_s  = '0; // _s for synchronised and idles low
    
    always_ff @(posedge clk) begin
        sck_s  <= {sck_s[1:0], mcu_sck};
    end

    wire sck_rise = (sck_s[2:1] == 2'b01);
    wire sck_fall = (sck_s[2:1] == 2'b10);

    logic [2:0] bit_cnt = '0; // where you are in the byte
    logic first_byte = 1'b1; // where you are in the byte
    logic [7:0] rx_sr  = '0;
    logic [7:0] sh_out  = '0;

    assign active = !cs_sync; // active when cs low

    wire [7:0] rx_next = {rx_sr[6:0], mosi_sync};
    wire byte_done = active && sck_rise && (bit_cnt == 3'd7);

    assign mcu_miso = sh_out[7];
    assign rx_byte = rx_next;
    assign rx_first = first_byte && byte_done;
    assign rx_valid = byte_done;
    assign tx_taken = active && sck_rise && (bit_cnt == 3'd0) && !first_byte;

    always_ff @(posedge clk) begin // if it was clocked on mcu_sck then you couldnt perfrom the active check
        if (!active) begin
            bit_cnt <= '0;
            rx_sr <= 0;
            sh_out <= '0;
            first_byte <= 1'b1;
        end else if (sck_rise) begin
            rx_sr <= rx_next;
            bit_cnt <= bit_cnt + 1'b1;
            if (bit_cnt == 3'd7) begin
                sh_out <= tx_byte;
                first_byte <= 1'b0;
            end

        end else if (sck_fall && bit_cnt != 3'd0) begin
            sh_out <= {sh_out[6:0], 1'b0};

        end
    end


endmodule 

// note that this drives miso always (not deep for this specific use but watch out for if multiple slaves on bus)