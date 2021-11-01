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

logic result_negative;
always_comb result_negative = difference[N-1];

mux4 #(.N(1)) SLT_MUX(
  .switch({a[31], b[31]}), // switch on the sign bits
  .in0(result_negative), .in1(1'b0), .in2(1'b1), .in3(result_negative),
  .out(out)
);

// SOLUTION END

endmodule