/*
An alternate implementation of an edge detector with Moore outputs (only a function of state).
*/

module edge_detector_moore(clk, rst, in, positive_edge, negative_edge);

input wire clk, rst, in;
output logic positive_edge, negative_edge;

enum logic [1:0] {S_LOW, S_POSITIVE_EDGE, S_HIGH, S_NEGATIVE_EDGE} state;

always @(posedge clk) begin
  if(rst) state <= S_LOW;
  else begin
    case (state)
      S_LOW: state <= in ? S_POSITIVE_EDGE : S_LOW;
      S_POSITIVE_EDGE : state <= in ? S_HIGH : S_NEGATIVE_EDGE;
      S_HIGH: state <= in ? S_HIGH : S_NEGATIVE_EDGE;
      S_NEGATIVE_EDGE : state <= in ? S_POSITIVE_EDGE : S_LOW;
    endcase
  end
end

always_comb begin : moore_output_logic
  positive_edge = (state == S_POSITIVE_EDGE);
  negative_edge = (state == S_NEGATIVE_EDGE);
end

endmodule