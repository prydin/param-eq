#pragma once

#include <RotaryEncoder.h>

class AcceleratedEncoder : public RotaryEncoder {
public:
    AcceleratedEncoder(int pin1, int pin2, LatchMode mode = LatchMode::FOUR3)
        : RotaryEncoder(pin1, pin2, mode), lastPosition(0), acceleratedPosition(0), minValue(0), maxValue(100) {}

    void setEndpoints(int minPos, int maxPos) {
        minValue = minPos;
        maxValue = maxPos;
    }

    void setAcceleratedPosition(int pos) {
        acceleratedPosition = pos;
        if (acceleratedPosition < minValue) {
            acceleratedPosition = minValue;
        }
        if (acceleratedPosition > maxValue) {
            acceleratedPosition = maxValue;
        }
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