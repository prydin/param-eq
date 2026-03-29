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
#include "display.h"

#include "audio_pipeline/audio_controller.h"
#include "netconv.h"
#include <algorithm>
#include <Wire.h>

namespace
{
constexpr uint32_t kStartupDisplaySampleRate = 44100;
}


void Display::sendRegister(uint8_t reg, uint32_t value, bool force)
{
  if(!force && value == registerCache[reg]) {
    return;
  }
  Wire.beginTransmission(DISPLAY_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write((uint8_t *)&value, sizeof(value));
  Wire.endTransmission();
  delay(1);
  registerCache[reg] = value;
}

void Display::setFilterBand(int band, bool force)
{
  sendRegister(REG_FILTER_SELECT, htonl(band), force); 
}

void Display::setFilterSettings(FilterSettings *settings, const sample_t *coeffs, bool force)
{
  if(!settings) {
    return;
  }
  sendRegister(REG_FILTER_TYPE, htonl(settings->type), force);
  sendRegister(REG_FILTER_FREQ, htonf(settings->frequency), force);
  sendRegister(REG_FILTER_Q, htonf(settings->Q), force);
  sendRegister(REG_FILTER_GAIN, htonf(settings->gain), force);
  sendRegister(REG_FILTER_COEFF0, htonf(coeffs[0]), force);
  sendRegister(REG_FILTER_COEFF1, htonf(coeffs[1]), force);
  sendRegister(REG_FILTER_COEFF2, htonf(coeffs[2]), force);
  sendRegister(REG_FILTER_COEFF3, htonf(-coeffs[3]), force);
  sendRegister(REG_FILTER_COEFF4, htonf(-coeffs[4]), force);

}

void Display::setDisplayMode(int displayMode, bool force)
{
  Serial.printf("Setting display mode to %d\n", displayMode);
  sendRegister(REG_DISPLAY_MODE, htonl(displayMode), force);
}

void Display::setMasterGain(float masterGain, bool force)
{
  Serial.printf("Setting master gain to %.2f\n", masterGain);
  sendRegister(REG_IN_GAIN, htonf(masterGain), force);
}

void Display::setVolume(float volume, bool force)
{
  sendRegister(REG_OUT_GAIN, htonf(volume), force);
}

void Display::setSampleRate(int sampleRate, bool force)
{
  sendRegister  (
      REG_SAMPLE_RATE,
      htonl(
          AudioController::isSampleRateStable()
              ? AudioController::getStandardizedSampleRate()
              : 0),
      force);
}

void Display::update(ControlValues &controlValues) {
  if(!controlValues.isDirty()) {
    return;
  }
  if(controlValues.isFilterChanged()) {
    if(controlValues.isBandChanged()) {
      setFilterBand(controlValues.getSelectedBand());
    }
    setFilterSettings(&controlValues.getCurrentFilter(), controlValues.getCoefficients());
  }
  setDisplayMode(controlValues.getDisplayMode());
  setMasterGain(controlValues.getMasterGain());
  setVolume(controlValues.getVolume());
  setUIMode(controlValues.getUIMode());
}

void Display::setVUMeterValue(float left, float right, bool force)
{
  const float clampedLeft = std::clamp(left, 0.0f, 1.0f);
  const float clampedRight = std::clamp(right, 0.0f, 1.0f);
  const uint16_t leftValue = static_cast<uint16_t>(clampedLeft * 65535.0f + 0.5f);
  const uint16_t rightValue = static_cast<uint16_t>(clampedRight * 65535.0f + 0.5f);
  uint32_t vuValue = (static_cast<uint32_t>(leftValue) << 16) | rightValue;
  sendRegister(REG_VU_METER, htonl(vuValue), force);
}

void Display::setUIMode(int uiMode, bool force)
{
  sendRegister(REG_UI_MODE, htonl(uiMode), force);
}

void Display::commit()
{
  sendRegister(REG_COMMIT, 1, true);
}
