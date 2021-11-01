# -Wall turns on all warnings
# -g2012 selects the 2012 version of iVerilog
IVERILOG=iverilog -Wall -g2012 -y ./ -I ./
VVP=vvp

# Look up .PHONY rules for Makefiles
.PHONY: clean

# Call this to clean up all your generated files
clean:
	rm -f *.bin *.vcd vivado*.log vivado*.jou vivado*.str

test_debouncer : test_debouncer.sv debouncer.sv
	${IVERILOG} $^ -o test_debouncer.bin && ${VVP} test_debouncer.bin

test_edge_detectors : test_edge_detectors.sv edge_detector.sv edge_detector_moore.sv
	${IVERILOG} $^ -o test_edge_detectors.bin && ${VVP} test_edge_detectors.bin