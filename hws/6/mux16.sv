module mux16(in0,in1,in2,in3,in4,in5,in6,in7,in8,in9,in10,in11,in12,in13,in14,in15,switch,out);

//parameter definitions
parameter N = 1;
//port definitions
input  wire [(N-1):0] in0;
input  wire [(N-1):0] in1;
input  wire [(N-1):0] in2;
input  wire [(N-1):0] in3;
input  wire [(N-1):0] in4;
input  wire [(N-1):0] in5;
input  wire [(N-1):0] in6;
input  wire [(N-1):0] in7;
input  wire [(N-1):0] in8;
input  wire [(N-1):0] in9;
input  wire [(N-1):0] in10;
input  wire [(N-1):0] in11;
input  wire [(N-1):0] in12;
input  wire [(N-1):0] in13;
input  wire [(N-1):0] in14;
input  wire [(N-1):0] in15;

input  wire [3:0] switch;
output logic [(N-1):0] out;

wire [(N-1):0] mux0, mux1;
//make 4:1 out of 2 8:1 muxes and a 2:1 mux
mux8 #(.N(N)) MUX_0 (.in0(in0), .in1(in1), .in2(in2), .in3(in3), .in4(in4), .in5(in5), .in6(in6), .in7(in7), .switch(switch[2:0]), .out(mux0));
mux8 #(.N(N)) MUX_1 (.in0(in8), .in1(in9), .in2(in10), .in3(in11), .in4(in12), .in5(in13), .in6(in14), .in7(in15), .switch(switch[2:0]), .out(mux1));
always_comb out = switch[3] ? mux1 : mux0;

endmodule
