# RotaryEncoder
## Another rotary encoder library for the Arduino

This yet another rotary encoder library package. It implements the class 
RotaryEncoder. A RotaryEncoder instance encapsulates a simple (cheap) incremental rotary 
encoder, typically used with a clicky rotating knob as a user input device. Most often they 
have three terminals, one referred to as A or CLK, another as B or DT. The third is GND. The 
A and B terminals each connect to one side of the A and B switches. The other ends of the 
switches are connected to the GND terminal. The A and B switches briefly close to mark each 
click of the encoder's shaft rotation. When the shaft is at rest, both switches are open. 
Encoders of this type also often have a switch that closes when the knob is pressed and opens 
when it is released. The push-button switch sometimes has separate terminals from the A and B 
switches and sometimes one end is connected to the GND terminal.

The code is designed to sense the closing of the switches -- A, B, and optionally the push 
button -- using microcontroller's the internal pull-up resistors to pull the pins to which the 
switches connect to Vcc when the switches are open.

The state of a RotaryEncoder object is typically kept current by polling. E.g., by having its 
run() member function invoked repeatedly in the sketch's loop() function. Button clicks are 
reported to the sketch via an event handler (aka callback function). Shaft rotation is reported 
either through an event handler or by the sketch asking for the current position, depending on 
the style that fits the sketch.
