module sync2 #(
    parameter logic INIT = 1'b0
) (
    input logic clk,
    input logic raw_data,
    output logic synchronised_data
);
    logic [1:0] data_s = {2{INIT}};

    always_ff @(posedge clk) begin
        data_s <= { data_s[0], raw_data};
    end
    assign synchronised_data = data_s[1];
    
endmodule