// Copyright (c) 2026 Pontus Rydin
// SPDX-License-Identifier: MIT
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

#include <RotaryEncoder.h>

class AcceleratedEncoder : public RotaryEncoder {
public:
    AcceleratedEncoder(int pin1, int pin2, LatchMode mode = LatchMode::FOUR3)
        : RotaryEncoder(pin1, pin2, mode), minValue(0), maxValue(100), lastPosition(0), acceleratedPosition(0) {}

    void setEndpoints(int minPos, int maxPos) {
        minValue = minPos;
        maxValue = maxPos;
    }

    void setAcceleratedPosition(int pos) {
        Serial.printf("Setting accelerated position to %d\n", pos);
        acceleratedPosition = pos;
        if (acceleratedPosition < minValue) {
            acceleratedPosition = minValue;
        }
        if (acceleratedPosition > maxValue) {
            acceleratedPosition = maxValue;
        }
        lastPosition = getPosition(); // Avoid spurious movements
    }

    int geAcceleratedPosition() const {
        return acceleratedPosition;
    }

    // Call this in your loop to update encoder position with acceleration
    void tick() {
        RotaryEncoder::tick();
        
        int newPos = getPosition();
        int acceleration = 1;
        if (newPos != lastPosition) {
            unsigned long timeDiff = getMillisBetweenRotations();
            
            // Calculate acceleration based on speed of rotation
            if (timeDiff < 50) {
                acceleration = 10;  // Very fast
            } else if (timeDiff < 100) {
                acceleration = 5;   // Fast
            } else if (timeDiff < 200) {
                acceleration = 2;   // Medium
            } else {
                acceleration = 1;   // Slow
            }
            
            Serial.printf("TimeDiff: %lu ms, Acceleration: %d\n", timeDiff, acceleration);
            Serial.printf("NewPos: %d, LastPos: %d\n", newPos, lastPosition);

            // Compute and update the new accelerated position  
            int delta = newPos - lastPosition;
            int newAcceleratedPosition = acceleratedPosition +  delta * acceleration;
            setAcceleratedPosition(newAcceleratedPosition);
            lastPosition = newPos;
        }
    }


private:
    int minValue;
    int maxValue;
    int lastPosition;
    int acceleratedPosition;
};  