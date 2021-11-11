`timescale 1ns/1ps
`default_nettype none

module mux32(in0,in1,in2,in3,in4,in5,in6,in7,in8,in9,in10,in11,in12,in13,in14,in15,
             in16,in17,in18,in19,in20,in21,in22,in23,in24,in25,in26,in27,in28,in29,in30,in31,
             switch,out);
	//parameter definitions
	parameter N = 1;
	//port definitions
	input  wire [(N-1):0] in0,in1,in2,in3,in4,in5,in6,in7,in8,in9,in10,in11,in12,in13,in14,in15,in16,in17,in18,in19,in20,in21,in22,in23,in24,in25,in26,in27,in28,in29,in30,in31;
	input  wire [4:0] switch;
	output logic [(N-1):0] out;
	
	wire [(N-1):0] mux0, mux1;
	//make 32:1 out of 2 16:1 muxes and a 2:1 mux
	mux16 #(.N(N)) MUX_0 (
		.in0(in0), .in1(in1), .in2(in2), .in3(in3), .in4(in4), .in5(in5), .in6(in6), .in7(in7), 
		.in8(in8), .in9(in9), .in10(in10), .in11(in11), .in12(in12), .in13(in13), .in14(in14), .in15(in15),
		.switch(switch[3:0]), .out(mux0)
	);
	mux16 #(.N(N)) MUX_1 (
		.in0(in16), .in1(in17), .in2(in18), .in3(in19), .in4(in20), .in5(in21), .in6(in22), .in7(in23), 
		.in8(in24), .in9(in25), .in10(in26), .in11(in27), .in12(in28), .in13(in29), .in14(in30), .in15(in31),
		.switch(switch[3:0]), .out(mux1)
	);
	always_comb out = switch[4] ? mux1 : mux0;

endmodule
`default_nettype wire
