`ifndef PARALELL_TO_SERIAL_H
`define PARALELL_TO_SERIAL_H

`default_nettype none


/*
Takes in an N-bit wide value on i_data. If the i_valid input is asserted copy data_in
to the internal shit register q. 

If i_valid is not asserted, the shift register starts outputting the values stored
based on the direction input which can be either MSB_FIRST or LSB_FIRST. 

I've implemented the MSB_FIRST logic for you, modify the blank sections to implement LSB_FIRST logic! 
You can do so in both sequential or combinational ways. 
*/

typedef enum logic [0:0] {MSB_FIRST, LSB_FIRST} shift_direction_t_;

module parallel_to_serial_converter(clk, rst, i_valid, direction, i_data, q, out);

parameter N = 4;

input wire clk, rst, i_valid;
input shift_direction_t_ direction;
input wire [N-1:0] i_data;
output logic out;
output logic [N-1:0] q; //shift register flip flops

always_ff @(posedge clk) begin
  if(rst) begin
    q <= 0;
  end else begin
    if (i_valid) begin
      q <= i_data;
    end else begin
      case(direction)
        MSB_FIRST: begin
          q[N-1:1] <= q[N-2:0];
          q[0] <= 0;
        end
        LSB_FIRST: begin
          q[N-2:0] <= q[N-1:1];
          q[N-1] <= 0;
        end
        default: q <= q;
      endcase
    end
  end
end

always_comb begin
  case(direction)
    MSB_FIRST: begin
      out = q[N-1];
    end
    LSB_FIRST: begin
      out = q[0];
    end
  endcase
end

endmodule

`endif 