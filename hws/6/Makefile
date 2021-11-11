# Note: this Makefile has some improvements over past assignments! Please read through this.
# Most noteable, we are switching from vcd waveforms to fst waveforms since they *might* 
# 	use less space (a lot of the class is more hard disk constrained than I expected).
#
# -Wall turns on all warnings
# -g2012 selects the 2012 version of iVerilog
IVERILOG=iverilog -Wall -Wno-sensitivity-entire-vector -Wno-sensitivity-entire-array -g2012 -y ./ -I ./
VVP=vvp
VVP_POST_ARGS=-fst

# Add your source sv files here! Is the fact that I populated this with muxes should be  *STRONG HINT*. 
SRCS=mux4.sv mux8.sv mux16.sv mux32.sv
# Look up .PHONY rules for Makefiles
.PHONY: clean submission

test_sll: test_sll.sv shift_left_logical.sv ${SRCS}
	${IVERILOG} $^ -o test_sll.bin && ${VVP} test_sll.bin ${VVP_POST_ARGS}

waves_sll: test_sll
		gtkwave sll.fst &

test_srl: test_srl.sv shift_right_logical.sv $(SRCS)
	${IVERILOG} $^ -o test_srl.bin && ${VVP} test_srl.bin ${VVP_POST_ARGS}

waves_srl: test_srl
		gtkwave srl.fst &

test_sra: test_sra.sv shift_right_arithmetic.sv $(SRCS)
	${IVERILOG} $^ -o test_srl.bin && ${VVP} test_srl.bin ${VVP_POST_ARGS}

waves_sra: test_sra
		gtkwave sra.fst &

test_parallel_to_serial_converter: test_parallel_to_serial_converter.sv parallel_to_serial_converter.sv
	${IVERILOG} $^ -o test_parallel_to_serial_converter.bin && ${VVP} test_parallel_to_serial_converter.bin ${VVP_POST_ARGS}

waves_parallel_to_serial_converter: test_parallel_to_serial_converter
		gtkwave parallel_to_serial_converter.fst --save=parallel_to_serial_converter.gtkw &


# Call this to clean up all your generated files
clean:
	rm -f *.bin *.vcd *.fst vivado*.log *.jou vivado*.str *.log *.checkpoint *.bit *.html *.xml

# Call this to generate your submission zip file.
submission:
	zip submission.zip Makefile *.sv *.pdf
