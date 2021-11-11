module slt(a, b, out);
parameter N = 32;
input wire signed [N-1:0] a, b;
output logic out;

// Using only *structural* combinational logic, make a module that computes if a is less than b!
// Note: this assumes that the two inputs are signed: aka should be interpreted as two's complement.

// Copy any other modules you use into this folder and update the Makefile accordingly.

// SOLUTION START
// Remember that you can make a subtractor with a 32 bit adder if you set the carry_in bit high, and invert one of the inputs.
logic [N-1:0] not_b;
always_comb not_b = ~b;
wire c_out;
wire [N-1:0] difference; 
adder_n #(.N(N)) SUBTRACTOR(
  .a(a), .b(not_b), .c_in(1'b1),
  .c_out(c_out), .sum(difference[N-1:0])
);

// The main trick in this problem is that we have to handle our subtractor's 
// outputs differently depending on what the signs of a and b are. There are 4
// possiblities (+,+), (+,-), (-,+), and (-,-), which screams truth table or 
// mux! I found the mux easier to implement since I already had one, but you 
// could also do a truth table -> sum of products approach.
// 
// (+,+) case: a is < b iff the result is negative
// (+,-) case: a is definitely greater than b, so out = 0
// (-,+) case: a is definitely less than b, so out = 1
// (-,-) case: a is < b iff the result is negative (same as first case)
// 
// The neatest thing about handling all of the cases this way is that we 
// don't need to worry about overflow conditions! Because the lagest possible 
// positive number is 1 less than the largest possible negative number you need
// the two operands to be the same sign to cause any issues.

`ifdef MUX_APPROACH

mux4 #(.N(1)) SLT_MUX(
  .switch({a[31], b[31]}), // switch on the sign bits
  .in0(difference[N-1]), .in1(1'b0), .in2(1'b1), .in3(difference[N-1]),
  .out(out)
);
`endif

`define CLEVER_GATES_APPROACH
`ifdef CLEVER_GATES_APPROACH

logic inputs_have_different_signs;
logic a_negative_b_positive;
logic a_positive_b_negative;
always_comb begin : slt_all_cases
  a_negative_b_positive = a[31] & ~ b[31];
  a_positive_b_negative = ~a[31] & b[31];
  inputs_have_different_signs = a[31] ^ b[31]; // unused, but kinda neat. 
  out = ~a_positive_b_negative & (a_negative_b_positive |  difference[31]);
end

`endif

// SOLUTION END

endmodule


