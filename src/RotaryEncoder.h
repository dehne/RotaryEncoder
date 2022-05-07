/****
 * @file     RotaryEncoder.h 
 *
 * This file is a part of yet another RotaryEncoder library package. It implements the class 
 * RotaryEncoder. A RotaryEncoder instance encapsulates a simple (cheap) incremental rotary 
 * encoder, typically used with a clicky rotating knob as a user input device. Most often they 
 * have three terminals, one referred to as A or CLK, another as B or DT. The third is GND. The 
 * A and B terminals each connect to one side of the A and B switches. The other ends of the 
 * switches are connected to the GND terminal. The A and B switches briefly close to mark each 
 * click of the encoder's shaft rotation. When the shaft is at rest, both switches are open. 
 * Encoders of this type also often have a switch that closes when the knob is pressed and opens 
 * when it is released. The push-button switch sometimes has separate terminals from the A and B 
 * switches and sometimes one end is connected to the GND terminal.
 * 
 * The code is designed to sense the closing of the switches -- A, B, and optionally the push 
 * button -- using microcontroller's the internal pull-up resistors to pull the pins to which the 
 * switches connect to Vcc when the switches are open.
 * 
 * How it works
 * ============
 * 
 * When the encoder's shaft is rotated in the clockwise direction, A (or CLK) closes, then B (or 
 * DT) closes then A opens, then B opens. Rotating counterclockwise causes B to close followed by 
 * A closing, followed by B opening followed by A opening. There is usually considerable 
 * glitchiness (switch bounce) in the opening and closing of the switches. There are various 
 * software- and hardware-based approaches to mitigating the bouncing. After trying out a bunch 
 * of them, I concluded that the easiest approach is a hybrid -- a little hardware and some care 
 * in the software. For the hardware part connect a 0.1uF or 0.2uF capacitor between the A and GND 
 * terminals and another between B and GND and a resistor of 100 - 200 ohms between the GND 
 * terminal and the Arduino's ground. If the encoder has a push-button switch, do the same for it.
 * 
 * To read the encoder the code detects the beginning of each A and each B closing and ignores the 
 * glitches. To do this, it exploits the fact that the A and B switch closures, and the attendant 
 * glitchiness that's not fixed by adding the capacitors, tends to be in quadrature. That is, the 
 * glitches the Arduino sees (almost) never occur at the same time on both switches. 
 * 
 * Central to the algorithm is a variable called nextSw. When nextSw is in state "a" the code 
 * looks for A to close. When nextSw is in state "b", it looks for B to close. It completely 
 * ignores switch closures it's not looking for, deeming them glitches. When a switch closure it 
 * *is* looking for actually happens, it changes nextSw to the other state, thus swapping which 
 * switch closure -- A or B -- it looks for. Based on testing, that gets rid of essentially all 
 * the glitchiness that gets past the hardwre mitigation. The closures it detects are the ones that 
 * mark the very start of the glitchy switch closures and virtually none of the glitches. 
 * 
 * The algorithm declares a rotational click when a detected switch closure happens and the other 
 * switch (which will be in a stable state) is closed. More specifically, on an A switch closure, 
 * a closed B indicates a click clockwise; on a B switch closure, a closed A indicates a click 
 * counterclockwise.
 * 
 * The push-button switch click detection works by detecting closures and measuring their length 
 * with the millis() clock. If a closure last less than 
 * 
 * The algorithm executes in the run() member function. You can use this to poll the state of the 
 * encoder in loop(). Or, alternatively, from a timer ISR, or, if your processor supportes is, using 
 * a pin-change ISR.
 *  
 * @version  Version 1.0.0, May 2022
 *
 * @author   D. L. Ehnebuske
 * 
 * @section  license
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022 by D. L. Ehnebuke All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   3. Neither the name of the copyright holders nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 ****/
#ifndef __ROTARY_ENCODER
    #define __ROTARY_ENCODER

    #include <Arduino.h>                                // Ritual Arduino include

    #define _RE_DEBUG                                   // Uncomment to enable debug output
    #define RE_LONG_MILLIS      (500)                   // Default for how many millis() make for a long click
    #define RE_MIN_MILLIS       (35)                    // Default for how many millis() a click must have to be noticed

    enum re_click_t : uint8_t {cShort, cLong};          // Type of click; short or long.
    extern "C" {                                        // The type a push-button click handler must have
        typedef void (*re_on_click_handler_t)(re_click_t click);
    }
    enum re_dir_t : uint8_t {cw, cc};                   // Type of rotation; clockwise or counterclockwise
    extern "C" {                                        // The type a rotation handler must have
        typedef void (*re_on_rotate_handler_t)(re_dir_t dir);
    }

    class RotaryEncoder {
        public:
            /****
             * 
             * RotaryEncoder(a, b[, s[, l[, m]]])
             *      The constructor; make a new instance of RotaryEncoder
             * 
             * Parameters:
             *      a   The pin to which the encoder's A or CLK pin is attached.
             *      b   The pin to which the encoder's B or DT pin is attached.
             *      s   The pin to which the push-button switch is attached. 0xFF = none.
             *      l   Minimum time in millis() for long click.
             *      m   Minimum time in millis() for a click to be considered "real".
             * 
             ****/
            RotaryEncoder(uint8_t a, uint8_t b, uint8_t s = 0xFF, uint16_t l = RE_LONG_MILLIS, uint16_t m = RE_MIN_MILLIS);

            /****
             * 
             * begin()
             *      Initialize the instance. Typically invoked once in setup(). 
             * 
             ****/
            void begin();

            /****
             * 
             * run()
             *      Poll the state of the A and B switches. Do this as frequently as possible.
             * 
             ****/
            void run();

            /****
             * 
             * attachOnRotation(handler)
             *      Attach the event handler for the rotation event. The rotation event handler  
             *      is invoked whenever the encoder's shaft is rotated on e step clockwise or 
             *      counterclockwise. When invoked, it is passed a re_dit_t parameter specifying 
             *      which direction the rotation was. The rotation is "consumed" in the sense that 
             *      the rotary encoder's position is moved one step towards 0 whenever the handler 
             *      is invoked.
             * 
             ****/
            void attachOnRotation(re_on_rotate_handler_t handler);

            /****
             * 
             * attachOnButton(handler)
             *      Attach the handler for the "switch clicked" event. The rotatio event handler 
             *      is invoked whenever the encoder's push button has been clicked, long-clicked 
             *      or double-clicked. When invoked, it is passed a re_click_t parameter 
             *      specifying what type of click has happened.
             * 
             ****/
            void attachOnButton(re_on_click_handler_t handler);

            /****
             * 
             * getPosition():   
             *      Return the current position of the rotary encoder in clicks. Positive numbers 
             *      are clockwise, negative counterclockwise.
             * 
             * Returns:
             *      The encoder's position
             * 
             ****/
            int16_t getPosition();

            /****
             * 
             * setPosition(p)   
             *      Set the current position, in clicks, to the passed value. Positive numbers are 
             *      clockwise, negative counterclockwise.
             * 
             *      Caution is needed when setting the encoder's position while also using a 
             *      rotation  handler (see attachOnRotation, above) since the handler will be 
             *      invoked repeatedly to reduce the position back to zero. It will work just 
             *      fine, but it's probably not what you're after.
             * 
             ****/
            void setPosition(int16_t p);

        private:
            uint8_t aPin;                               // The GPIO pin the encoder's A switch is attached to
            uint8_t bPin;                               // The GPIO pin the encoder's B switch is attached to
            uint8_t sPin;                               // The GPIO pin the encoder's push-button switch is attached to 0xFF = none
            re_on_rotate_handler_t rHandler = nullptr;  // The rotation event handler
            re_on_click_handler_t cHandler = nullptr;   // The click event handler
            bool aWasOpen = true;                       // true if A was open the last time we looked, false otherwise
            bool bWasOpen = true;                       // true if B was open the last time we checked, false otherwise
            bool sWasOpen = true;                       // true if push-button was open last we checked, false othewise
            unsigned long sMillis = 0;                  // millis() last time we checked push-button
            unsigned long sMinMillis;                   // How many millis() a click must take to be noticed
            unsigned long sLongMillis;                  // How many millis() make for a long click

            enum re_nextsw_t : uint8_t {switchA, switchB};  
            volatile re_nextsw_t nextSw = switchA;      // Whether the next switch to close is A or B
            volatile int16_t position = 0;              // Net position + is clockwise; - is counterclockwise
    };

#endif