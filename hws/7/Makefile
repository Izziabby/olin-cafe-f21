# Note: this Makefile has some improvements over past assignments! Please read through this.
# Most noteable, we are switching from vcd waveforms to fst waveforms since they *might* 
# 	use less space (a lot of the class is more hard disk constrained than I expected).
#
# -Wall turns on all warnings
# -g2012 selects the 2012 version of iVerilog
IVERILOG=iverilog -Wall -Wno-sensitivity-entire-vector -Wno-sensitivity-entire-array -g2012 -y ./ -I ./
VVP=vvp
VVP_POST=-fst

# Add your source sv files here! Is the fact that I populated this with muxes should be  *STRONG HINT*. 
SRCS=mux4.sv mux8.sv mux16.sv mux32.sv \
	shift_left_logical.sv shift_right_logical.sv shift_right_arithmetic.sv \
	adder_1.sv adder_n.sv
# Look up .PHONY rules for Makefiles
.PHONY: clean submission

test_alu: test_alu.sv alu.sv alu_types.sv alu_behavioural.sv ${SRCS}
	${IVERILOG} $^ -o test_alu.bin && ${VVP} test_alu.bin ${VVP_POST}

waves_alu: test_alu
		gtkwave alu.fst &

# Call this to clean up all your generated files
clean:
	rm -f *.bin *.vcd *.fst vivado*.log *.jou vivado*.str *.log *.checkpoint *.bit *.html *.xml

# Call this to generate your submission zip file.
submission:
	zip submission.zip Makefile *.sv *.pdf
