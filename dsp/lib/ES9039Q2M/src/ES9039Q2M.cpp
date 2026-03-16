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
#include "ES9039Q2M.h"

ES9039Q2M::ES9039Q2M() {
    // Constructor - set default I2C address
    _i2cAddress = ES9039Q2M_I2C_ADDR;
}

ES9039Q2M::~ES9039Q2M() {
    // Destructor - nothing specific to clean up
}

bool ES9039Q2M::begin(uint8_t address) {
    // Store the I2C address
    _i2cAddress = address;
    
    // Initialize I2C communication
    Wire.begin();
    return true;
}

uint8_t ES9039Q2M::readRegister8(uint8_t reg) {
    Wire.beginTransmission(_i2cAddress);
    Wire.write(reg);
    
    uint32_t status = Wire.endTransmission(false);
    if (status != 0) {
        Serial.println("Error: I2C transmission failed. Status: " + String(status));
        return 0; // Error occurred
    }
    
    Wire.requestFrom(_i2cAddress, (uint8_t)1);
    
    if (Wire.available()) {
        return Wire.read();
    }
    
    return 0; // No data available
}

void ES9039Q2M::writeRegister8(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_i2cAddress | 0x80); // Set write flag
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void ES9039Q2M::writeRegisterMasked8(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t current = readRegister8(reg);
    current = (current & ~mask) | (value & mask);
    writeRegister8(reg, current);
}

uint16_t ES9039Q2M::readRegister16(uint8_t reg) {
    Wire.beginTransmission(_i2cAddress);
    Wire.write(reg);
    
    if (Wire.endTransmission(false) != 0) {
        return 0; // Error occurred
    }
    
    Wire.requestFrom(_i2cAddress, (uint8_t)2);
    
    if (Wire.available() >= 2) {
        uint16_t value = Wire.read() << 8;
        value |= Wire.read();
        return value;
    }
    
    return 0; // No data available
}

void ES9039Q2M::writeRegister16(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(_i2cAddress);
    Wire.write(reg);
    Wire.write(uint8_t((value >> 8) & 0xFF));
    Wire.write(uint8_t(value & 0xFF));
    Wire.endTransmission();
}
    
void ES9039Q2M::writeRegisterMasked16(uint8_t reg, uint16_t mask, uint16_t value) {
    uint16_t current = readRegister16(reg);
    current = (current & ~mask) | (value & mask);
    writeRegister16(reg, current);
}

int32_t ES9039Q2M::readRegister24Signed(uint8_t reg) {
    Wire.beginTransmission(_i2cAddress);
    Wire.write(reg);
    
    if (Wire.endTransmission(false) != 0) {
        return 0; // Error occurred
    }
     
    Wire.requestFrom(_i2cAddress, (uint8_t)3);
    
    if (Wire.available() >= 3) {
        int32_t value = Wire.read() << 16;
        value |= Wire.read() << 8;
        value |= Wire.read();
        // Sign extend 24-bit value to 32-bit signed integer
        if (value & 0x800000) {
            value |= 0xFF000000;
        }
        return value;
    }
    
    return 0; // No data available
}

void ES9039Q2M::writeRegister24Signed(uint8_t reg, int32_t value) {
    Wire.beginTransmission(_i2cAddress);
    Wire.write(reg);
    Wire.write(uint8_t((value >> 16) & 0xFF));
    Wire.write(uint8_t((value >> 8) & 0xFF));
    Wire.write(uint8_t(value & 0xFF));
    Wire.endTransmission();
}

  void ES9039Q2M::setVolume(float volume) {
    float volf = log10f(volume) * 20.0f; // Convert linear volume to dB scale
    if(volf < -127.5f) volf = -127.5f;
    if(volf > 0.0f) volf = 0.0f;
    uint8_t vol = uint8_t((-volf) * 2.0f); // Convert dB to register value (0.5 dB steps)
    //Serial.printf("Setting volume: %f (dB: %f, reg: %d)\n", volume, volf, vol);
    writeRegister8(ES9039Q2M_REG_VOLUME_CH1, vol);
    writeRegister8(ES9039Q2M_REG_VOLUME_CH2, vol);
  }

    void setFilterParameters(uint8_t filterType, uint8_t filterValue) {
     // TODO: Implement filter parameter setting
    }
    
    void ES9039Q2M::setVolumeDB(float volumeDB)
    {
        if (volumeDB > 0.0f) volumeDB = 0.0f;
        if (volumeDB < -127.5f) volumeDB = -127.5f;
        uint8_t volume = static_cast<uint8_t>(-2.0f * volumeDB);
        setCH1Volume(volume);
        setCH2Volume(volume);
    }
