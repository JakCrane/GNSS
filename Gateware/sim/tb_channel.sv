// Plays a recorded capture through one tracking channel (open loop: fixed fwords)
// and prints the six correlator sums at every dump.
//
// Driven by Sims/channel_test.py, which writes the sample file and passes:
//   +SAMPLES=<file>  +N=<count>  +PRN=<n>  +CARR=<carrier fword>  +CODE=<code fword>  +START=<clock to release reset>
`timescale 1ns/1ps
module tb_channel;
    logic clk = 0, reset = 1;
    logic signed [2:0] sample = 0;
    logic [5:0]  prn;
    logic [31:0] carr_fword, code_fword;

    logic signed [19:0] ie, ip, il, qe, qp, ql;
    logic dump, early, prompt, late, epoch;

    channel dut (
        .clk(clk), .reset(reset), .prn(prn),
        .code_fword(code_fword), .carrier_fword(carr_fword), .sample(sample),
        .ie(ie), .ip(ip), .il(il), .qe(qe), .qp(qp), .ql(ql),
        .dump(dump), .early(early), .prompt(prompt), .late(late), .epoch(epoch)
    );

    always #30.55 clk = ~clk;   // ~16.368 MHz (exact period doesn't matter in simulation)

    logic [2:0] mem [0:262143];
    string fname;
    int n, start, p, carr, code;

    initial begin
        if (!$value$plusargs("SAMPLES=%s", fname)) $fatal(1, "need +SAMPLES=");
        void'($value$plusargs("N=%d", n));
        void'($value$plusargs("PRN=%d", p));
        void'($value$plusargs("CARR=%d", carr));
        void'($value$plusargs("CODE=%d", code));
        void'($value$plusargs("START=%d", start));
        prn = p; carr_fword = carr; code_fword = code;
        $readmemh(fname, mem);

        for (int t = 0; t < n; t++) begin
            sample = mem[t];
            if (t == start) reset = 0;
            @(posedge clk);
            #1;
        end
        $finish;
    end

    // The sums update on the dump edge, so print them one clock later.
    int k = 0;
    logic dump_d = 0;
    always @(posedge clk) begin
        if (dump_d) begin
            $display("DUMP %0d  IE %0d QE %0d  IP %0d QP %0d  IL %0d QL %0d", k, ie, qe, ip, qp, il, ql);
            k++;
        end
        dump_d <= dump && !reset;
    end
endmodule
