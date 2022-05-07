/****
 * @file     RotaryEncoder.cpp
 * 
 * This file is a part of the RotaryEncoder library package. See the file RotaryEncoder.h 
 * for details.
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
#include "RotaryEncoder.h"                          // Our definitions
#include <util/atomic.h>                            // For access to ATOMIC_BLOCK macro
#include <avr/io.h>                                 // For access to INTx definitions

#ifdef _RE_DEBUG
#define _RE_RED_LED     (6)
#define _RE_YELLOW_LED  (5)
#define _RE_GREEN_LED   (4)
#endif


RotaryEncoder::RotaryEncoder(uint8_t a, uint8_t b, uint8_t s, uint16_t l, uint16_t m) {
    aPin = a;
    bPin = b;
    sPin = s;
    sLongMillis = l;
    sMinMillis = m;
}

void RotaryEncoder::begin() {
  #ifdef _RE_DEBUG
  pinMode(_RE_RED_LED, OUTPUT);
  pinMode(_RE_YELLOW_LED, OUTPUT);
  pinMode(_RE_GREEN_LED, OUTPUT);
  digitalWrite(_RE_RED_LED, LOW);
  digitalWrite(_RE_YELLOW_LED, LOW);
  digitalWrite(_RE_GREEN_LED, LOW);
  #endif

  // Set up the switch pins
  pinMode(aPin, INPUT_PULLUP);
  pinMode(bPin, INPUT_PULLUP);
  if (sPin != 0xFF) {
    pinMode(sPin, INPUT_PULLUP);
  }
  #ifdef _RE_DEBUG
  Serial.print(F("RotaryEncoder on pins "));
  Serial.print(aPin);
  Serial.print(F(", "));
  Serial.print(bPin);
  Serial.print(F(", "));
  Serial.println(sPin);
  #endif
}

void RotaryEncoder::run() {

  // Deal with rotation
  bool aIsClosed = digitalRead(aPin) == LOW;
  bool bIsClosed = digitalRead(bPin) == LOW;
  if (nextSw == switchA) {              // If looking for A to close
    if (aIsClosed && aWasOpen) {        //   If it just closed
      if (bIsClosed) {                  //     If B is closed, we have one step clockwise
        position++;
      }
      nextSw = switchB;                 //   Change to looking for B to close
    }
  } else {                              // Otherwise looking for B to close
    if (bIsClosed && bWasOpen) {        //   If it just closed
      if (aIsClosed) {                  //     If A is closed, we have one step counterclockwise
        position--;
      }
      nextSw = switchA;                 //   Change to looking for A to close
    }
  }
  aWasOpen = !aIsClosed;                // Remember most recent state of A and B
  bWasOpen = !bIsClosed;

  // Invoke rotation event handler, if appropriate
  if (rHandler != nullptr && position != 0) {
    re_dir_t dir = cw;
    if (position > 0) {
      position --;
    } else {
      position++;
      dir = cc;
    }
    (*rHandler)(dir);
  }

  // Deal with push-button
  if (sPin != 0xFF && cHandler != nullptr) {    // If there's a valid button even handler
    bool sIsClosed = digitalRead(sPin) == LOW;
    unsigned long nowMillis = millis();
    #ifdef _RE_DEBUG
    digitalWrite(_RE_RED_LED, sIsClosed ? HIGH : LOW);
    #endif
    if (sWasOpen && sIsClosed) {                // If button just closed
      sMillis = nowMillis;                      //   Remember when it happened 
    } else if (!sWasOpen && !sIsClosed && nowMillis - sMillis > sMinMillis) {       
                                                // Otherwise if it just opened and it's not a glitch; invoke the handler
      (*cHandler)(nowMillis - sMillis >= sLongMillis ? cLong : cShort);
    }
    sWasOpen = !sIsClosed;                      // Remember the latest state
  }
}

void RotaryEncoder::attachOnRotation(re_on_rotate_handler_t handler) {
  rHandler = handler;
}

void RotaryEncoder::attachOnButton(re_on_click_handler_t handler) {
  cHandler = handler;
}

int16_t RotaryEncoder::getPosition() {
  return position;
}

void RotaryEncoder::setPosition(int16_t p) {
  position = p;
}
