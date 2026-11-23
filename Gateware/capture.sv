module capture #(
    parameter int DEPTH = 32768
) (
    input  logic       clk,
    input  logic [1:0] sample,
    input  logic       start,
    input  logic       ready_next,
    output logic       busy,
    output logic [7:0] rdata
);
    localparam int AW = $clog2(DEPTH);
    logic [7:0] mem [0:DEPTH-1];

    logic [AW-1:0] waddr  = '0;
    logic [AW-1:0] raddr  = '0;
    logic [1:0]    phase  = '0;
    logic [5:0]    pack   = '0;
    logic          busy_r = 1'b0;

    wire we = busy_r && (phase == 2'd3);
    assign busy = busy_r;

    always_ff @(posedge clk) begin
        if (start) begin
            busy_r <= 1'b1;
            waddr  <= '0;
            phase  <= '0;
        end else if (busy_r) begin
            phase <= phase + 1'b1;
            pack  <= {pack[3:0], sample};
            if (phase == 2'd3) begin
                waddr <= waddr + 1'b1;
                if (waddr == AW'(DEPTH-1)) busy_r <= 1'b0;
            end
        end
    end

    always_ff @(posedge clk) begin
        if (we) mem[waddr] <= {pack, sample};
    end

    always_ff @(posedge clk) begin
        if (start)           raddr <= '0;
        else if (ready_next) raddr <= raddr + 1'b1;
        rdata <= mem[raddr];
    end
endmodule