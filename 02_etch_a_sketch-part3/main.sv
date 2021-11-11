`default_nettype none

/*
  Etch-a-sketch lab: Part 2: Touch input.
*/

`include "ft6206_defines.sv"

module main(
  // On board signals
  clk, buttons, leds, rgb, pmod,
  // Display signals
  interface_mode,
  touch_i2c_scl, touch_i2c_sda, touch_irq,
  backlight, display_rstb, data_commandb,
  display_csb, spi_mosi, spi_miso, spi_clk
);
parameter CLK_HZ = 12_000_000; // aka ticks per second
parameter CLK_PERIOD_NS = (1_000_000_000/CLK_HZ); // Approximation.
parameter PWM_PERIOD_US = 100; 
parameter PWM_WIDTH = 12;
parameter PERIOD_MS_FADE = 100;
parameter PWM_TICKS = CLK_HZ*PWM_PERIOD_US/1_000_000; //1kHz modulation frequency. // Always multiply before dividing, it avoids truncation.
parameter human_divider = 23; // A clock divider parameter - 12 MHz / 2^23 is about 1 Hz (human visible speed).

//Module I/O and parameters
input wire clk;
input wire [1:0] buttons;
logic rst; always_comb rst = buttons[0]; // Use button 0 as a reset signal.
output logic [1:0] leds;
output logic [2:0] rgb;
output logic [7:0] pmod;  always_comb pmod = 0; // You can use the pmod port for debugging!

// Display driver signals
output wire [3:0] interface_mode;
output wire touch_i2c_scl;
inout wire touch_i2c_sda;
input wire touch_irq;
output wire backlight, display_rstb, data_commandb;
output wire display_csb, spi_clk, spi_mosi;
input wire spi_miso;

assign backlight = 1;
ili9341_display_controller ILI9341(
  .clk(clk), .rst(rst), .ena(1'b1), .display_rstb(display_rstb), .interface_mode(interface_mode),
  .spi_csb(display_csb), .spi_clk(spi_clk), .spi_mosi(spi_mosi), .spi_miso(spi_miso),
  .data_commandb(data_commandb)
);


// Some useful timing signals. //TODO@(avinash) - move to a different module or use a generate to save space here...
wire step_1Hz;
pulse_generator #(.N($clog2(CLK_HZ/1))) PULSE_1Hz (
  .clk(clk), .rst(rst), .ena(1'b1), .out(step_1Hz),
  .ticks(CLK_HZ/1)
);

wire step_10Hz;
pulse_generator #(.N($clog2(CLK_HZ/10))) PULSE_10Hz (
  .clk(clk), .rst(rst), .ena(1'b1), .out(step_10Hz),
  .ticks(CLK_HZ/10)
);

wire step_100Hz;
pulse_generator #(.N($clog2(CLK_HZ/100))) PULSE_100Hz (
  .clk(clk), .rst(rst), .ena(1'b1), .out(step_100Hz),
  .ticks(CLK_HZ/100)
);


// capacitive touch controller
touch_t touch0, touch1;
ft6206_controller #(.CLK_HZ(CLK_HZ), .I2C_CLK_HZ(100_000)) FT6206(
  .clk(clk), .rst(rst), .ena(step_100Hz),
  .scl(touch_i2c_scl), .sda(touch_i2c_sda),
  .touch0(touch0), .touch1(touch1)
);

// LED PWM logic.
logic [PWM_WIDTH-1:0] led_pwm0, led_pwm1;
always @(posedge clk) begin
  if(rst) begin
    led_pwm0 <= 0;
    led_pwm1 <= 0;
  end else begin
    if(touch0.valid) begin
      led_pwm0 <= touch0.x;
      led_pwm1 <= touch0.y;
    end
  end
end

pwm #(.N(PWM_WIDTH)) PWM_LED0 (
  .clk(clk), .rst(rst), .ena(1'b1), .step(1'b1), .duty(led_pwm0),
  .out(leds[0])
);

pwm #(.N(PWM_WIDTH)) PWM_LED1 (
  .clk(clk), .rst(rst), .ena(1'b1), .step(1'b1), .duty(led_pwm1),
  .out(leds[1])
);

endmodule

`default_nettype wire // reengages default behaviour, needed when using 
                      // other designs that expect it.