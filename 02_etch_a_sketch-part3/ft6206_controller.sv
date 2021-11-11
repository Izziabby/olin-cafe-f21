`include "ft6206_defines.sv"
`include "i2c_types.sv"

`timescale 1ns/1ps
`default_nettype none

module ft6206_controller(clk, rst, ena, scl, sda, touch0, touch1);

parameter CLK_HZ = 12_000_000;
parameter CLK_PERIOD_NS = (1_000_000_000/CLK_HZ);
parameter I2C_CLK_HZ = 400_000; // Must be <= 400kHz
parameter DIVIDER_COUNT = CLK_HZ/I2C_CLK_HZ/2;  // Divide by two necessary since we toggle the signal

parameter DEFAULT_THRESHOLD = 128;
parameter N_RD_BYTES = 16;

// Module I/O and parameters
input wire clk, rst, ena;
output wire scl;
inout wire sda;
output touch_t touch0, touch1;

i2c_transaction_t i2c_mode;
wire i_ready;
logic i_valid;
FT6206_register_t i_data;
logic o_ready;
wire o_valid;
wire [7:0] o_data;


i2c_controller #(.CLK_HZ(CLK_HZ), .I2C_CLK_HZ(I2C_CLK_HZ)) I2C0 (
  .clk(clk), .rst(rst), 
  .scl(scl), .sda(sda),
  .mode(i2c_mode), .i_ready(i_ready), .i_valid(i_valid), .i_addr(`FT6206_ADDRESS), .i_data(i_data),
  .o_ready(o_ready), .o_valid(o_valid), .o_data(o_data)
);

// Main fsm
enum logic [3:0] {
  S_IDLE = 0,
  S_INIT = 1, // Unused for now
  S_WR_START_DATA_BURST = 2,
  S_RD_DATA = 3,
  S_WAIT_FOR_I2C_WR = 4,
  S_WAIT_FOR_I2C_RD = 5,
  S_ERROR
} state, state_after_wait;

logic [3:0] num_touches;
touch_t touch0_buffer, touch1_buffer;
logic [$clog2(N_RD_BYTES):0] bytes_counter;

always_ff @(posedge clk) begin
  if(rst) begin
    state <= S_IDLE;
    state_after_wait <= S_IDLE;
    bytes_counter <= 0;
    // TODO(avinash) - merge touch0 and touch1 buffers, can get away with less state that way.
    touch0_buffer <= 0;
    touch1_buffer <= 0;
    touch0 <= 0;
    touch1 <= 0;
  end else begin
    case(state)
      S_IDLE : begin
        if(i_ready & ena)
          state <= S_WR_START_DATA_BURST;
      end
      S_WR_START_DATA_BURST : begin
        state <= S_WAIT_FOR_I2C_WR;
        state_after_wait <= S_RD_DATA;
        bytes_counter <= 0;
      end
      S_RD_DATA : begin
        state_after_wait <= S_RD_DATA;
        if(o_valid) begin
          // Parse the resulting data into the right structure based on byte number
          $display("  buffer[%d] = 0x%h", bytes_counter, o_data);
          case(bytes_counter)
            8'h1 : begin
              touch0_buffer.gesture <= o_data;
              touch1_buffer.gesture <= o_data;
            end
            8'h2 : begin
              num_touches <= o_data[3:0];
              if(o_data[3:0] == 4'd1 || o_data[3:0] == 4'd2) begin
                touch0_buffer.valid <= 1;
              end
              if(o_data[3:0] == 4'd2) begin
                touch1_buffer.valid <= 1;
              end
            end
            8'h3 : begin
              $display("!!! x[11:8] = ", o_data[3:0]);
              touch0_buffer.x[11:8] <= o_data[3:0];
              touch0_buffer.contact <= o_data[7:6];
            end
            8'h4 : begin
              touch0_buffer.x[7:0] <= o_data;
              $display("!!! x[7:0] = ", o_data);
            end
            8'h5 : begin
              $display("!!!touch_buffer.x = %d", touch0_buffer.x);
              touch0_buffer.y[11:8] <= o_data[3:0];
              touch0_buffer.id <= o_data[7:4];
            end
            8'h6 : touch0_buffer.y[7:0] <= o_data;
            8'h7 : touch0_buffer.weight <= o_data;
            8'h8 : touch0_buffer.area <= o_data[7:4];
            8'h9 : begin
              touch1_buffer.x[11:8] <= o_data[3:0];
              touch1_buffer.contact <= o_data[7:6];
            end
            8'ha : touch1_buffer.x[7:0] <= o_data;
            8'hb : begin
              touch1_buffer.y[11:8] <= o_data[3:0];
              touch1_buffer.id <= o_data[7:4];
            end
            8'hc : touch1_buffer.y[7:0] <= o_data;
            8'hd : touch1_buffer.weight <= o_data;
            8'he : touch1_buffer.area <= o_data[7:4];
          endcase
          
        end
        if(bytes_counter < N_RD_BYTES) begin
          state <= S_WAIT_FOR_I2C_RD;
          bytes_counter <= bytes_counter + 1;
        end
        else begin
          $display("Copying touch buffer 0...");
          print_touch(touch0_buffer);
          $display("Copying touch buffer 1...");
          print_touch(touch1_buffer);
          touch0 <= touch0_buffer;
          touch1 <= touch1_buffer;
          state <= S_IDLE;
        end
      end
      S_WAIT_FOR_I2C_WR : begin
        if(i_ready) state <= state_after_wait;
      end
      S_WAIT_FOR_I2C_RD : begin
        if(i_ready & o_valid) state <= state_after_wait;
      end
    endcase
  end
end

always_comb case(state)
  S_WR_START_DATA_BURST, S_RD_DATA: i_valid = 1;
  default: i_valid = 0;
endcase 

always_comb case(state)
  S_WR_START_DATA_BURST : i2c_mode = WRITE_8BIT_REGISTER;
  S_RD_DATA: i2c_mode = READ_8BIT;
  default: i2c_mode = WRITE_8BIT_REGISTER;
endcase


always_comb case(state)
  S_WR_START_DATA_BURST: i_data = DEVICE_MODE;
  default: i_data = DEVICE_MODE;
endcase
endmodule


