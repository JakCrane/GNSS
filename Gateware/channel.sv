module channel (
    input  logic                clk,
    input  logic                reset,
    input  logic [5:0]          prn,
    input  logic [31:0]         code_fword,
    input  logic [31:0]         carrier_fword,
    input  logic signed [2:0]   sample,    
    output logic signed [19:0]  ie, ip, il, qe, qp, ql,   
    output logic                dump, 
    output logic                early,
    output logic                prompt, // now (as in not late not early)
    output logic                late,
    output logic                epoch,
    input  logic [15:0]         slew,
    input  logic                slew_load
);

    logic [15:0] slew_left = '0;

    always_ff @(posedge clk) begin
        if (slew_load) begin           
            slew_left <= slew;
        end else if (slew_left != 0) begin
            slew_left <= slew_left - 1'b1;
        end 
    end

    logic chip_en;
    logic chip;
    logic [31:0] phase;

    logic [15:0] chip_array = '0;
    code_nco nco (
        .clk     (clk),
        .reset   (reset),
        .fword   (code_fword),
        .chip_en (chip_en),
        .phase   (phase),
        .enable  (slew_left == 0)
    );

    ca_gen ca (
        .clk (clk),
        .reset (reset),
        .chip_en (chip_en),
        .prn (prn),
        .chip (chip),
        .epoch (epoch)
    );

    always_ff @(posedge clk) begin
        chip_array <= {chip_array[14:0], chip};
    end

    assign early = chip;
    assign prompt = chip_array[7];
    assign late = chip_array[15];

    logic epoch_d = 1'b0;                      // epoch, one clock ago

    always_ff @(posedge clk)
        epoch_d <= epoch;

    wire dump_now = epoch && !epoch_d;

    logic signed [3:0] i_mix, q_mix;

    carrier_mixer mix (
        .clk(clk), 
        .reset(reset), 
        .sample(sample),
        .carrier_fword(carrier_fword), 
        .i_mix(i_mix), 
        .q_mix(q_mix)
    );

    wire signed [3:0] ie_now = early  ? -i_mix : i_mix;
    wire signed [3:0] ip_now = prompt ? -i_mix : i_mix;
    wire signed [3:0] il_now = late   ? -i_mix : i_mix;

    wire signed [3:0] qe_now = early  ? -q_mix : q_mix;
    wire signed [3:0] qp_now = prompt ? -q_mix : q_mix;
    wire signed [3:0] ql_now = late   ? -q_mix : q_mix;

    logic signed [19:0] ie_acc = '0, ip_acc = '0, il_acc = '0;
    logic signed [19:0] qe_acc = '0, qp_acc = '0, ql_acc = '0;
    logic signed [19:0] ie_out = '0, ip_out = '0, il_out = '0;
    logic signed [19:0] qe_out = '0, qp_out = '0, ql_out = '0;

    always_ff @(posedge clk) begin
        if (reset) begin
            ie_acc <= '0; ip_acc <= '0; il_acc <= '0;
            qe_acc <= '0; qp_acc <= '0; ql_acc <= '0;
        end else if (dump_now) begin
            ie_out <= ie_acc + ie_now;  ie_acc <= '0;
            ip_out <= ip_acc + ip_now;  ip_acc <= '0;
            il_out <= il_acc + il_now;  il_acc <= '0;
            qe_out <= qe_acc + qe_now;  qe_acc <= '0;
            qp_out <= qp_acc + qp_now;  qp_acc <= '0;
            ql_out <= ql_acc + ql_now;  ql_acc <= '0;
        end else begin
            ie_acc <= ie_acc + ie_now;
            ip_acc <= ip_acc + ip_now;
            il_acc <= il_acc + il_now;
            qe_acc <= qe_acc + qe_now;
            qp_acc <= qp_acc + qp_now;
            ql_acc <= ql_acc + ql_now;
        end
    end

    assign ie = ie_out;  assign ip = ip_out;  assign il = il_out;
    assign qe = qe_out;  assign qp = qp_out;  assign ql = ql_out;
    assign dump = dump_now;

endmodule