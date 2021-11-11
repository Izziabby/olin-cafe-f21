`timescale 1ns/1ps
`default_nettype none
module shift_left_logical(in, shamt, out);

parameter N = 32; // only used as a constant! Don't feel like you need to a shifter for arbitrary N.

input wire [N-1:0] in;            // the input number that will be shifted left. Fill in the remainder with zeros.
input wire [$clog2(N)-1:0] shamt; // the amount to shift by (think of it as a decimal number from 0 to 31). 
output logic [N-1:0] out;       

// SOLUTION START

/*
Totally fine to manually write this, but scripting can be your friend! Here's some python:
  print("  " + "\n  ".join([f".in{i}({{in[N-{i}-1:0], {i}'b0}})," for i in range(1, 32)]))
*/
mux32 #(.N(32)) SLL_MUX (
  .in0(in),
  .in1({in[N-1-1:0], 1'b0}),
  .in2({in[N-2-1:0], 2'b0}),
  .in3({in[N-3-1:0], 3'b0}),
  .in4({in[N-4-1:0], 4'b0}),
  .in5({in[N-5-1:0], 5'b0}),
  .in6({in[N-6-1:0], 6'b0}),
  .in7({in[N-7-1:0], 7'b0}),
  .in8({in[N-8-1:0], 8'b0}),
  .in9({in[N-9-1:0], 9'b0}),
  .in10({in[N-10-1:0], 10'b0}),
  .in11({in[N-11-1:0], 11'b0}),
  .in12({in[N-12-1:0], 12'b0}),
  .in13({in[N-13-1:0], 13'b0}),
  .in14({in[N-14-1:0], 14'b0}),
  .in15({in[N-15-1:0], 15'b0}),
  .in16({in[N-16-1:0], 16'b0}),
  .in17({in[N-17-1:0], 17'b0}),
  .in18({in[N-18-1:0], 18'b0}),
  .in19({in[N-19-1:0], 19'b0}),
  .in20({in[N-20-1:0], 20'b0}),
  .in21({in[N-21-1:0], 21'b0}),
  .in22({in[N-22-1:0], 22'b0}),
  .in23({in[N-23-1:0], 23'b0}),
  .in24({in[N-24-1:0], 24'b0}),
  .in25({in[N-25-1:0], 25'b0}),
  .in26({in[N-26-1:0], 26'b0}),
  .in27({in[N-27-1:0], 27'b0}),
  .in28({in[N-28-1:0], 28'b0}),
  .in29({in[N-29-1:0], 29'b0}),
  .in30({in[N-30-1:0], 30'b0}),
  .in31({in[N-31-1:0], 31'b0}),
  .switch(shamt),
  .out(out)
);


// SOLUTION END

endmodule
