module debouncer(clk, rst, bouncy_in, debounced_out);
parameter BOUNCE_TICKS = 10;
input wire clk, rst;
input wire bouncy_in;

output logic debounced_out;

always_comb debounced_out = 1'bx; // just for debugging - will show up red!

enum logic [1:0] {
    S_0 = 2'b00,
    S_MAYBE_1 = 2'b01,
    S_1 = 2'b10,
    S_MAYBE_0 = 2'b11
} state;

//clog2 = celling log base 2 = how many bits do i need
logic [$clog2(BOUNCE_TICKS):0] counter;

always_comb begin : output_logic
    case(state)
     S_0, S_MAYBE_1 : begin
      debounced_out = 0;
     end
      S_1, S_MAYBE_0 : debounced_out = 1;
     default: debounced_out = 1'bx;
    endcase
end

// above equivelent to always_comb debounced_out = state[1]
always_ff @(posedge clk ) begin : fsm_logic
    if (rst) begin
        state <= S_0;
    end else begin
        case(state)
            S_0 : begin
               if (bouncy_in) begin
                   counter <= 0;
                   state <= S_MAYBE_1;
               end
            end

            S_MAYBE_1 : begin
                counter <= counter + 1;
                if (counter == (BOUNCE_TICKS - 1)) begin
                    if (bouncy_in) state<= S_1;
                    else state <= S_0;
                end
            end

            S_1 : begin
                if (~bouncy_in) begin
                    counter <= 0;
                    state <= S_MAYBE_0;
                end
            end

            S_MAYBE_0 : begin
                counter <= counter + 1;
                if (counter == (BOUNCE_TICKS - 1)) begin
                    if (bouncy_in) state<= S_1;
                    else state <= S_0;
                end
            end
            default : state <= S_0
        endcase
end

endmodule