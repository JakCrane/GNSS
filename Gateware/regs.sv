module regs (
    input  logic       clk,
    // from mcu_spi
    input  logic [7:0] rx_byte,
    input  logic       rx_valid,
    input  logic       rx_first,
    output logic [7:0] tx_byte,
    // things to report
    input  logic       ld,
    input  logic       cap_busy,

    input  logic       tx_taken,
    input  logic [7:0] cap_rdata,
    output logic       cap_start,
    output logic       cap_next
);
    localparam logic [6:0] A_ID      = 7'h00;
    localparam logic [6:0] A_STATUS  = 7'h01;
    localparam logic [6:0] A_CTRL    = 7'h02;   // write bit 0 = 1 → start capture
    localparam logic [6:0] A_CAPDATA = 7'h03;   // each read = next captured byte

    logic [6:0] addr = '0;
    wire        hdr_write = rx_byte[7];
    wire  [6:0] hdr_addr  = rx_byte[6:0];
    logic       is_write  = 1'b0;            // remembered from the header

    wire [6:0] next_addr = (addr == A_CAPDATA) ? addr : addr + 1'b1;
    wire [6:0] rd_addr   = rx_first ? hdr_addr : next_addr;

    wire wr = rx_valid && !rx_first && is_write;

    wire  reading = rx_first ? !hdr_write : !is_write;
    logic loaded_capdata = 1'b0;

    always_ff @(posedge clk) begin
        if (rx_first) begin
            addr     <= hdr_addr;            // byte 1 belongs to the header's address
            is_write <= hdr_write;
        end else if (rx_valid) begin
            addr <= next_addr;             // move to the next slot
        end
        if (rx_valid) loaded_capdata <= reading && (rd_addr == A_CAPDATA);
    end

    always_comb begin
        case (rd_addr)
            A_ID:      tx_byte = 8'h5A;
            A_STATUS:  tx_byte = {6'b0, cap_busy, ld};
            A_CAPDATA: tx_byte = cap_rdata;
            default:   tx_byte = 8'h00;
        endcase
    end

    assign cap_start = wr && (addr == A_CTRL) && rx_byte[0];
    assign cap_next = tx_taken && loaded_capdata;


endmodule