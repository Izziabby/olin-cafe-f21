`include "spi_types.sv"

module spi_controller(
  clk, rst, sclk, csb, mosi, miso,
  spi_mode, i_ready, i_valid, i_data, o_ready, o_valid, o_data,
  bit_counter
);

input wire clk, rst; // default signals.

// SPI Signals
output logic sclk; // Serial clock to secondary device.
output logic csb; // chip select bar, needs to go low at the start of any SPI transaction, then go high whne done.
output logic mosi; // Main Out Secondary In (sends serial data to secondary device)
input wire miso; // Main In Secondary Out (receives serial data from secondary device)

// Control Signals
input spi_transaction_t spi_mode; // The mode (see spi_types.sv) of the transaction. We only need to implement WRITE8 and WRITE16
output logic i_ready; // handshake signals
input wire i_valid;
input wire [15:0] i_data; // the input data that will be sent over SPI.

input wire o_ready; // Unused for now.
output logic o_valid; // hand shake signals
output logic [23:0] o_data; // the received data from the SPI bus. Not needed for this lab.
output logic unsigned [4:0] bit_counter; // the number of the current bit being transmit (should go from MSB down to 0).

// TX : transmitting
// RX: receiving
enum logic [2:0] {S_IDLE, S_TXING, S_TX_DONE, S_RXING, S_RX_DONE, S_ERROR } state;

// Internal registers/counters
logic [15:0] tx_data; // copy i_data into here when you are starting a transaction.
logic [23:0] rx_data; // fill this in from MISO, not required for this lab.

/*
This is going to be one of our more complicated FSMs. 
We need to sample inputs on the positive edge of sclk, but 
we also want to set outputs on the negative edge of the clk (it's
  the safest time to change an output given unknown peripheral
  setup/hold times).

To do this we are going to toggle sclk every cycle. We can then test
whether we are about to be on a negative edge or a positive edge by 
checking the current value of sclk. If it's 1, we're about to go negative,
so that's a negative edge.

Combinationally:
- we want to drive csb so that it goes low at the start of each transaction
  and then high again before the next one.
- we also want to drive mosi based on the current bit (bit_counter) and tx_data
*/


always_comb begin : csb_logic
end

always_comb begin : mosi_logic
end

always_ff @(posedge clk) begin : spi_controller_fsm
  if(rst) begin
    state <= S_IDLE;
    sclk <= 0;
    bit_counter <= 0;
    o_valid <= 0;
    i_ready <= 1;
    tx_data <= 0;
    rx_data <= 0;
    o_data <= 0;
  end else begin
    case(state)
      S_IDLE : begin
      end
      S_TXING : begin
        sclk <= ~sclk;
        // positive edge logic
        if(~sclk) begin
        end else begin // negative edge logic
        end
      end
      S_TX_DONE : begin
        // Next State Logic
        case (spi_mode)
          WRITE_8, WRITE_16: begin
              state <= S_IDLE;
              i_ready <= 1;
          end
          default : state <= S_RXING;
        endcase
        // Bit Counter Reset Logic
        case (spi_mode)
          WRITE_8_READ_8  : bit_counter <= 5'd7;
          WRITE_8_READ_16 : bit_counter <= 5'd15;
          WRITE_8_READ_24 : bit_counter <= 5'd23;
          default : bit_counter <= 0;
        endcase
      end

      S_RXING : begin
        sclk <= ~sclk;
        if(~sclk) begin // positive edge logic
        end else begin // negative edge logic
        end
      end

      S_RX_DONE: begin
        
      end
      default : state <= S_ERROR;
    endcase
  end
end

endmodule
