/****
 * @file     main.cpp
 * 
 * This file is an example Arduino sketch showing how to use the RotaryEncoder library package. 
 * See the file ../../../src/RotaryEncoder.h for details.
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
#include <Arduino.h>
#include <RotaryEncoder.h>

#define A_PIN     (3)                         // Pin to which the rotary encoder's "A" switch is attached
#define B_PIN     (2)                         // Pin to which the rotary encoder's "B" switch is attached
#define S_PIN     (7)                         // Pin to which the rotary encoder's push-button is attached

#define BANNER    F("\nRotary encoder position tracker example v1.0.0")

RotaryEncoder re {A_PIN, B_PIN, S_PIN};       // Object encapsulating the rotary encoder on A_PIN, B_PIN, S_PIN

void onButton(re_click_t type) {
  if (type == cShort) {
    Serial.print(F("Position: "));
    Serial.println(re.getPosition());
  } else {
    re.setPosition(0);
    Serial.println(F("Position reset to 0."));
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println(BANNER);
  Serial.print(F("Turn knob to set position. Click to display position. Long-click to reset position to 0."));

  re.begin();                                 // Initialize the rotary encoder
  re.attachOnButton(onButton);                // Attach its button event handler
}

void loop() {
  re.run();                                   // Let the RotaryEncoder do its thing
}