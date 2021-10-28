`timescale 1ns/1ps
`default_nettype none
`include "parallel_to_serial_converter.sv"

module test_parallel_to_serial_register;
parameter N = 8;

logic clk, rst, i_valid;
shift_direction_t_ direction;
logic [N-1:0] i_data;
wire out;
wire [N-1:0] q;

parallel_to_serial_converter #(.N(N)) UUT (
  .clk(clk), .rst(rst), .i_valid(i_valid), .direction(direction), 
  .i_data(i_data), .out(out), .q(q)
);

task print_state;
  $display("out = %b, q = %b", out, q);
endtask

task set_state(input logic [N-1:0]  d);
  @(negedge clk);
  i_data = d;
  i_valid = 1;
  @(negedge clk);
  i_valid = 0;
endtask

always #5 clk = ~clk;

initial begin
  $dumpfile("parallel_to_serial_converter.fst");
  $dumpvars(0, UUT);
  clk = 0;
  i_valid = 0;
  rst = 1;
  i_data = 0;
  direction = MSB_FIRST;

  repeat (2) @(posedge clk);
  rst = 0;
  $display("MSB First test...");
  set_state(8'b1010_1010);
  for (int i = 0; i <= N; i = i + 1) begin
    @(posedge clk) print_state();
  end

  $display("##########################################");
  $display("LSB First Test...");

  direction=LSB_FIRST;
  set_state(8'b1010_1010);
  for (int i = 0; i <= N; i = i + 1) begin
    @(posedge clk) print_state();
  end

  repeat (N) @(posedge clk);

  $finish;
end



endmodule