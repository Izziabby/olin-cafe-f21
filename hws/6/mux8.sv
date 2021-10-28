module mux8(in0,in1,in2,in3,in4,in5,in6,in7,switch,out);

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
input  wire [2:0] switch;
output logic [(N-1):0] out;

wire [(N-1):0] mux0, mux1;
//make 4:1 out of 2 4:1 muxes and a 2:1 mux
mux4 #(.N(N)) MUX_0 (.in0(in0), .in1(in1), .in2(in2), .in3(in3), .switch(switch[1:0]), .out(mux0));
mux4 #(.N(N)) MUX_1 (.in0(in4), .in1(in5), .in2(in6), .in3(in7), .switch(switch[1:0]), .out(mux1));
always_comb out = switch[2] ? mux1 : mux0;

endmodule
