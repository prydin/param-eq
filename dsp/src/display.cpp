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

Display::Display(TwoWire &wire,
                 AudioFilterBiquadFloat &filter,
                 FilterSettings *filterSettings,
                 int &selectedFilterBand,
                 int &displayMode,
                 float &masterGain,
                 float &volume,
                 uint16_t &displayChangeBitmap,
                 const uint8_t &uiMode)
    : wire(wire),
      filter(filter),
      filterSettings(filterSettings),
      selectedFilterBand(selectedFilterBand),
      displayMode(displayMode),
      masterGain(masterGain),
      volume(volume),
      displayChangeBitmap(displayChangeBitmap),
      uiMode(uiMode)
{
}

bool Display::updateDisplayRegister(uint8_t reg, uint32_t value)
{
  wire.beginTransmission(DISPLAY_I2C_ADDRESS);
  wire.write(reg);
  wire.write((uint8_t *)&value, sizeof(value));
  uint8_t status = wire.endTransmission();
  if (status != 0)
  {
    Serial.printf("Display I2C write failed (reg=0x%02X, status=%u)\n", reg, status);
    return false;
  }
  delay(1);
  return true;
}

void Display::commit()
{
  updateDisplayRegister(REG_COMMIT, 1);
}

void Display::updateSampleRate()
{
  updateDisplayRegister(
      REG_SAMPLE_RATE,
      htonl(
          AudioController::isSampleRateStable()
              ? AudioController::getStandardizedSampleRate()
              : 0));
  commit();
}

void Display::updateDisplay()
{
  uint16_t changed = displayChangeBitmap;
  if (changed == 0)
  {
    return;
  }

  // Full sync: send all bands to display
  if (changed & DISPLAY_CHANGE_FULL_FILTER_SYNC)
  {
    for (int band = 0; band < FILTER_BANDS; band++)
    {
      updateDisplayRegister(REG_FILTER_SELECT, htonl(band));
      updateDisplayRegister(REG_FILTER_TYPE, htonl(filterSettings[band].type));
      updateDisplayRegister(REG_FILTER_FREQ, htonf(filterSettings[band].frequency));
      updateDisplayRegister(REG_FILTER_Q, htonf(filterSettings[band].Q));
      updateDisplayRegister(REG_FILTER_GAIN, htonf(filterSettings[band].gain));

      const sample_t *coeffs = filter.getCoefficients(band);
      updateDisplayRegister(REG_FILTER_COEFF0, htonf(coeffs[0]));
      updateDisplayRegister(REG_FILTER_COEFF1, htonf(coeffs[1]));
      updateDisplayRegister(REG_FILTER_COEFF2, htonf(coeffs[2]));
      updateDisplayRegister(REG_FILTER_COEFF3, htonf(-coeffs[3]));
      updateDisplayRegister(REG_FILTER_COEFF4, htonf(-coeffs[4]));
    }
    updateDisplayRegister(REG_FILTER_SELECT, htonl(selectedFilterBand));
  }

  // Incremental updates for selected band
  if (changed & DISPLAY_CHANGE_FILTER_SELECT)
  {
    updateDisplayRegister(REG_FILTER_SELECT, htonl(selectedFilterBand));
  }

  if (changed & DISPLAY_CHANGE_DISPLAY_MODE)
  {
    updateDisplayRegister(REG_DISPLAY_MODE, htonl(displayMode));
  }

  if (changed & DISPLAY_CHANGE_FILTER_TYPE)
  {
    updateDisplayRegister(REG_FILTER_TYPE, htonl(filterSettings[selectedFilterBand].type));
  }

  if (changed & DISPLAY_CHANGE_FILTER_COEFFS)
  {
    const sample_t *coeffs = filter.getCoefficients(selectedFilterBand);
    updateDisplayRegister(REG_FILTER_COEFF0, htonf(coeffs[0]));
    updateDisplayRegister(REG_FILTER_COEFF1, htonf(coeffs[1]));
    updateDisplayRegister(REG_FILTER_COEFF2, htonf(coeffs[2]));
    updateDisplayRegister(REG_FILTER_COEFF3, htonf(-coeffs[3]));
    updateDisplayRegister(REG_FILTER_COEFF4, htonf(-coeffs[4]));
  }

  if (changed & DISPLAY_CHANGE_FILTER_FREQ)
  {
    updateDisplayRegister(REG_FILTER_FREQ, htonf(filterSettings[selectedFilterBand].frequency));
  }

  if (changed & DISPLAY_CHANGE_FILTER_Q)
  {
    updateDisplayRegister(REG_FILTER_Q, htonf(filterSettings[selectedFilterBand].Q));
  }

  if (changed & DISPLAY_CHANGE_FILTER_GAIN)
  {
    updateDisplayRegister(REG_FILTER_GAIN, htonf(filterSettings[selectedFilterBand].gain));
  }

  if (changed & DISPLAY_CHANGE_IN_GAIN)
  {
    updateDisplayRegister(REG_IN_GAIN, htonf(masterGain));
  }

  if (changed & DISPLAY_CHANGE_OUT_GAIN)
  {
    updateDisplayRegister(REG_OUT_GAIN, htonf(volume));
  }

  if (changed & DISPLAY_CHANGE_UI_MODE)
  {
    updateDisplayRegister(REG_UI_MODE, htonl(static_cast<uint32_t>(uiMode)));
  }

  // Send sample rate and commit
  updateSampleRate();
  displayChangeBitmap = 0;
}
