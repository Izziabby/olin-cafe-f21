`include "ili9341_defines.sv"
`include "spi_types.sv"
`include "ft6206_defines.sv"

/*
Display controller for the ili9341 chip on Adafruit's breakout baord.
Based on logic from: https://github.com/adafruit/Adafruit_ILI9341

*/

module ili9341_display_controller(
  clk, rst, ena, display_rstb,
  interface_mode,
  spi_csb, spi_clk, spi_mosi, spi_miso, data_commandb,
  vsync, hsync,
  touch,
  vram_rd_addr, vram_rd_data
);

parameter CLK_HZ = 12_000_000; // aka ticks per second
parameter DISPLAY_WIDTH = 240;
parameter DISPLAY_HEIGHT = 320;
parameter VRAM_L = DISPLAY_HEIGHT*DISPLAY_WIDTH;
parameter CFG_CMD_DELAY = CLK_HZ*150/1000; // wait 150ms after certain configuration commands
parameter ROM_LENGTH=125; // Set this based on the output of generate_memories.py

input wire clk, rst, ena;
output logic display_rstb; // Need a separate value because the display has an opposite reset polarity.
always_comb display_rstb = ~rst; // Fix the active low reset

// SPI Interface
output logic spi_csb, spi_clk, spi_mosi; //chip select, clock, main out secondary in
input wire spi_miso; // main in secondary out

// Sets the mode (many parallel and serial options, see page 10 of the datasheet).
output logic [3:0] interface_mode;
always_comb interface_mode = 4'b1110; // Standard SPI 8-bit mode is 4'b1110.

output logic data_commandb; // Set to 1 to send data, 0 to send commands. Read as Data/Command_Bar

output logic vsync; // Should combinationally be high for one clock cycle when drawing the last pixel (239,319)
output logic hsync; // Should combinationally be high for one clock cycle when drawing the last pixel of any row (x = 239).

input touch_t touch; // Current touch event. 

input ILI9341_color_t vram_rd_data; // VRAM rd data
output logic [$clog2(VRAM_L)-1:0] vram_rd_addr; // VRAM rd addr.

// SPI Controller that talks to the ILI9341 chip
spi_transaction_t spi_mode; //set mode (read or write)
wire i_ready; //ready to accept input
logic i_valid; //data is available
logic [15:0] i_data; //input data
logic o_ready; //ready to output
wire o_valid; //data is available
wire [23:0] o_data; //output data
wire [4:0] spi_bit_counter; //counter
spi_controller SPI0(
    .clk(clk), .rst(rst), 
    .sclk(spi_clk), .csb(spi_csb), .mosi(spi_mosi), .miso(spi_miso),
    .spi_mode(spi_mode), .i_ready(i_ready), .i_valid(i_valid), .i_data(i_data),
    .o_ready(o_ready), .o_valid(o_valid), .o_data(o_data),
    .bit_counter(spi_bit_counter)
);

// ROM that stores the configuration sequence the display needs
wire [7:0] rom_data; //data from rom
logic [$clog2(ROM_LENGTH)-1:0] rom_addr; //rom adress
block_rom #(.INIT("memories/ili9341_init.memh"), .W(8), .L(ROM_LENGTH)) ILI9341_INIT_ROM (
  .clk(clk), .addr(rom_addr), .data(rom_data)
);


// Main FSM
enum logic [2:0] {
  S_INIT = 0,
  S_INCREMENT_PIXEL = 1,
  S_START_FRAME = 2,
  S_TX_PIXEL_DATA_START = 3,
  S_TX_PIXEL_DATA_BUSY = 4,
  S_WAIT_FOR_SPI = 5,
  S_ERROR //very useful for debugging
} state, state_after_wait;

// Configuration FSM
enum logic [2:0] {
  S_CFG_GET_DATA_SIZE = 0,
  S_CFG_GET_CMD = 1,
  S_CFG_SEND_CMD = 2,
  S_CFG_GET_DATA = 3,
  S_CFG_SEND_DATA = 4,
  S_CFG_SPI_WAIT = 5,
  S_CFG_MEM_WAIT = 6,
  S_CFG_DONE
} cfg_state, cfg_state_after_wait;

ILI9341_color_t pixel_color;
logic [$clog2(DISPLAY_WIDTH):0] pixel_x; //bit size based on width of screen
logic [$clog2(DISPLAY_HEIGHT):0] pixel_y; //bit size based on height of screen

ILI9341_register_t current_command; 

// Comb. outputs
/* Note - it's pretty critical that you keep always_comb blocks small and separate.
   there's a weird order of operations that can mess up your synthesis or simulation.  
*/

always_comb case(state)
  S_START_FRAME, S_TX_PIXEL_DATA_START : i_valid = 1; //can send data in these states
  S_INIT : begin
    case(cfg_state)
      S_CFG_SEND_CMD, S_CFG_SEND_DATA: i_valid = 1;
      default: i_valid = 0;
    endcase
  end
  default: i_valid = 0; //default state data not available
endcase
  
always_comb case (state) 
  S_START_FRAME : current_command = RAMWR; //write ram in this state
  default : current_command = NOP;
endcase

always_comb case(state)
  S_INIT: i_data = {8'd0, rom_data}; //makes pixel color bit 0 in this state
  S_START_FRAME: i_data = {8'd0, current_command}; //makes current command bit 0 in this state
  default: i_data = pixel_color; //data is pixel_color
endcase

always_comb case (state)
  S_INIT, S_START_FRAME: spi_mode = WRITE_8; //set spi mode as 8 bit counter for this state
  default : spi_mode = WRITE_16; //default set spi mode as 16 bit counter
endcase

always_comb begin
  hsync = pixel_x == (DISPLAY_WIDTH-1); //high if pixel is in last column
  vsync = hsync & (pixel_y == (DISPLAY_HEIGHT-1)); // high if pixel is in last row and hsync (bottom right corner)
end




always_comb begin  : draw_cursor_logic
  vram_rd_addr = pixel_y*DISPLAY_WIDTH + pixel_x; //set read address for pixel

    //if touch location is the same as the location being written to draw a 6x6 square
  if(touch.valid & (touch.x[8:2] == pixel_x[8:2]) 
    & (touch.y[8:2] == pixel_y[8:2])) begin
    //pixel_color = WHITE; //color of square being drawn
    pixel_color = PINK; //my implimentation (PINK square)
  end else begin
    pixel_color = vram_rd_data; //have this draw from memory
  end
end

logic [$clog2(CFG_CMD_DELAY):0] cfg_delay_counter; //clock divider
logic [7:0] cfg_bytes_remaining;

always_ff @(posedge clk) begin : main_fsm
    //set to initial state (0) when rst
  if(rst) begin
    state <= S_INIT;
    cfg_state <= S_CFG_GET_DATA_SIZE;
    cfg_state_after_wait <= S_CFG_GET_DATA_SIZE;
    cfg_delay_counter <= 0;
    state_after_wait <= S_INIT;
    pixel_x <= 0;
    pixel_y <= 0;
    rom_addr <= 0;
    data_commandb <= 1;
  end
  else if(ena) begin
    case (state)
      S_INIT: begin
        case (cfg_state)
          S_CFG_GET_DATA_SIZE : begin
            cfg_state_after_wait <= S_CFG_GET_CMD; //get command after wait
            cfg_state <= S_CFG_MEM_WAIT; //setting state
            rom_addr <= rom_addr + 1; //move to next pixel
            case(rom_data) 
              8'hFF: begin //data address full
                cfg_bytes_remaining <= 0; //no more to read
                cfg_delay_counter <= CFG_CMD_DELAY; //set delay
              end
              8'h00: begin //data address empty
                cfg_bytes_remaining <= 0; //no more to read
                cfg_delay_counter <= 0; //set delay to 0
                cfg_state <= S_CFG_DONE; //set state to DONE
              end
              default: begin
                cfg_bytes_remaining <= rom_data; //default read rom data
                cfg_delay_counter <= 0; //default delay of 0
              end
            endcase
          end
          S_CFG_GET_CMD: begin
            cfg_state_after_wait <= S_CFG_SEND_CMD; //send command after getting
            cfg_state <= S_CFG_MEM_WAIT; //wait 
          end
          S_CFG_SEND_CMD : begin
            data_commandb <= 0; //send to display
              //if no rom data set state to DONE
            if(rom_data == 0) begin
              cfg_state <= S_CFG_DONE;
            end else begin //otherwise wait for and get data
              cfg_state <= S_CFG_SPI_WAIT;
              cfg_state_after_wait <= S_CFG_GET_DATA;
            end
          end
          S_CFG_GET_DATA: begin
            data_commandb <= 1; //send to display
            rom_addr <= rom_addr + 1; //move to next pixel
              //continue to send data until there are no more bytes left to send
            if(cfg_bytes_remaining > 0) begin
              cfg_state_after_wait <= S_CFG_SEND_DATA;
              cfg_state <= S_CFG_MEM_WAIT;
              cfg_bytes_remaining <= cfg_bytes_remaining - 1;
            end else begin //once zero bytes, get config data size
              cfg_state_after_wait <= S_CFG_GET_DATA_SIZE;
              cfg_state <= S_CFG_MEM_WAIT;
            end
          end
          S_CFG_SEND_DATA: begin //get data and wait for spi
            cfg_state_after_wait <= S_CFG_GET_DATA;
            cfg_state <= S_CFG_SPI_WAIT;
          end
          S_CFG_DONE : begin //config is done, move to start frame
            state <= S_START_FRAME; // S_TX_PIXEL_DATA_START; //TODO@(avinash)
          end
          S_CFG_SPI_WAIT : begin //remains in this state until config counter = 0
            if(cfg_delay_counter > 0) cfg_delay_counter <= cfg_delay_counter-1;
            else if (i_ready) begin
               cfg_state <= cfg_state_after_wait;
               cfg_delay_counter <= 0;
               data_commandb <= 1;
            end
          end
          S_CFG_MEM_WAIT : begin
            // If you had a memory with larger or unknown latency you would put checks in this state to wait till the data was ready.
            cfg_state <= cfg_state_after_wait;
          end
          default: cfg_state <= S_CFG_DONE;
        endcase
      end
      S_WAIT_FOR_SPI: begin
        if(i_ready) begin //only continue once ready to accept data
          state <= state_after_wait;
        end
      end
      S_START_FRAME: begin
        data_commandb <= 0; //send commands to display
        state <= S_WAIT_FOR_SPI; //check if ready
        state_after_wait <= S_TX_PIXEL_DATA_START; //then start
      end
      S_TX_PIXEL_DATA_START: begin
        data_commandb <= 1; //send data to display
        state_after_wait <= S_INCREMENT_PIXEL; //increment after wait
        state <= S_WAIT_FOR_SPI; //check if ready 
      end
      S_TX_PIXEL_DATA_BUSY: begin
        if(i_ready) state <= S_INCREMENT_PIXEL; //if ready to accept, increment pixel
      end
      S_INCREMENT_PIXEL: begin
        state <= S_TX_PIXEL_DATA_START; //sends data to display and increments pixel states
        if(pixel_x < (DISPLAY_WIDTH-1)) begin //moves to the right
          pixel_x <= pixel_x + 1;
        end else begin
          pixel_x <= 0; //reset to left edge
          if (pixel_y < (DISPLAY_HEIGHT-1)) begin //moves down
            pixel_y <= pixel_y + 1;
          end else begin
            pixel_y <= 0; //reset to top edge (ends up being top right corner bc of the other reset)
            state <= S_START_FRAME; // S_TX_PIXEL_DATA_START; //TODO(avinash)
          end
        end
      end
      default: begin
        state <= S_ERROR; //default nothing happens
        pixel_y <= -1; //invalid pixel values
        pixel_x <= -1;
      end
    endcase
  end
end

endmodule