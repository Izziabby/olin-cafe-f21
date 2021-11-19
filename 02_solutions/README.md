# Lab 2: Etch a Sketch

In this lab we're going to build logic to make an "etch a sketch" or sketchpad hardware device. Over the course of the lab you will learn how to:
* Design your own specifications for complex sequential and combinational logic blocks.
* Implement controllers for the popular SPI and i2c serial interfaces.
* Learn how to interface with both ROM and RAM memories.
* Get better at 

We're using [Adafruit's 2.8" TFT LCD with Cap Touch Breakout Board w/MicroSD Socket](https://www.adafruit.com/product/2090). Through the course of the lab we'll interface with following components on the breakout board:
- a 240x320 RGB TFT Display
- an ILI9341 Display Controller [datasheet](https://cdn-shop.adafruit.com/datasheets/ILI9341.pdf)
- an FT6206 Capacitive Touch Controller [datasheet](https://cdn-shop.adafruit.com/datasheets/FT6x06+Datasheet_V0.1_Preliminary_20120723.pdf) and [app note](https://cdn-shop.adafruit.com/datasheets/FT6x06_AN_public_ver0.1.3.pdf)
- (stretch) an SD Card

## Lab Checklist

- [ ] Pulse generator
- [ ] PWM Module
- [ ] General purpose parallel to serial module
- [ ] General purpose serial to parallel module
- [ ] SPI Controller for Display
- [ ] i2c Controller for touchscreen
- [ ] main system FSM 
  - [ ] clear memory on button press
  - [ ] update memory based on touch values
  - [ ] emit draw signals based on memory
  - [ ] bonus: add colors, different modes
  - [ ] stretch bonus: add fonts/textures!

# A) Sequential Logic & FPGA Programming
Let's start with a simple example to make sure we all have the tools working and can effectively design, simulate, and synthesize combination logic and simple FSMs.

## Pulse Generator
Start by trying to implement a pulse generator - this is a module that outputs high for exactly one clock cycle out of every N ticks. Implement your code in `pulse_generator.sv`. Note that the corresponding testbench is now in a `tests` folder to fix some odd FPGA bitstream generation issues I noticed last time.

Get an instructor sign off by showing your working gtkwave simulation before proceeding!

## PWM Module
Pulse Width Modulation, or PWM is the first and easiest way of trying to get an analog or continuous value from a digital signal. Design and simulate an implementation in `pwm.sv`. Like before, show your working simulation in gtkwave before proceeding.

## Triangle Generator
A triangle or sawtooth generator is a counter that starts at zero, counts up to its maximum value, then counts down back to zero, etc. Implement a simple FSM, and show your waveforms to an instructor before proceeding.

## Putting it all together
Last, we're going to showcase the three above modules in `main.sv` by fading the LEDs in and out. To do this we'll use `pulse_generators` to generate some slower "step" signals that keep things changing at human rates. Next we'll use our `triangle_generator` to make a signal we can use to brighten and dim our LEDs. Finally, the `pwm` modules actually drive the LEDs!

All of this has been implemented in `main.sv` so you shouldn't have to make any changes. Start by doing a `make main.bit`, then use either `make program_fpga_vivado` or `make program_fpga_digilent` to program the FPGA. I've tested the digilent version less but it might work where the vivado one fails.




