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
                 uint16_t &displayChangeBitmap)
    : wire(wire),
      filter(filter),
      filterSettings(filterSettings),
      selectedFilterBand(selectedFilterBand),
      displayMode(displayMode),
      masterGain(masterGain),
      volume(volume),
      displayChangeBitmap(displayChangeBitmap)
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
  static bool fullSyncDone = false;
  uint16_t changed = displayChangeBitmap;
  if (changed == 0)
  {
    return;
  }

  Serial.println("Updating display with changed filter settings...");

  uint16_t before = changed;
  const uint16_t perFilterMask = DISPLAY_CHANGE_FILTER_SELECT |
                                 DISPLAY_CHANGE_FILTER_TYPE |
                                 DISPLAY_CHANGE_FILTER_FREQ |
                                 DISPLAY_CHANGE_FILTER_Q |
                                 DISPLAY_CHANGE_FILTER_GAIN |
                                 DISPLAY_CHANGE_FILTER_COEFFS;

  if (!fullSyncDone && (changed & DISPLAY_CHANGE_FULL_FILTER_SYNC))
  {
    bool fullSyncOk = true;
    for (int band = 0; band < FILTER_BANDS; band++)
    {
      fullSyncOk = fullSyncOk && updateDisplayRegister(REG_FILTER_SELECT, htonl(band));
      fullSyncOk = fullSyncOk && updateDisplayRegister(REG_FILTER_TYPE, htonl(filterSettings[band].type));
      fullSyncOk = fullSyncOk && updateDisplayRegister(REG_FILTER_FREQ, htonf(filterSettings[band].frequency));
      fullSyncOk = fullSyncOk && updateDisplayRegister(REG_FILTER_Q, htonf(filterSettings[band].Q));
      fullSyncOk = fullSyncOk && updateDisplayRegister(REG_FILTER_GAIN, htonf(filterSettings[band].gain));

      const sample_t *coeffs = filter.getCoefficients(band);
      fullSyncOk = fullSyncOk && updateDisplayRegister(REG_FILTER_COEFF0, htonf(coeffs[0]));
      fullSyncOk = fullSyncOk && updateDisplayRegister(REG_FILTER_COEFF1, htonf(coeffs[1]));
      fullSyncOk = fullSyncOk && updateDisplayRegister(REG_FILTER_COEFF2, htonf(coeffs[2]));
      fullSyncOk = fullSyncOk && updateDisplayRegister(REG_FILTER_COEFF3, htonf(-coeffs[3]));
      fullSyncOk = fullSyncOk && updateDisplayRegister(REG_FILTER_COEFF4, htonf(-coeffs[4]));

      if (!fullSyncOk)
      {
        break;
      }
    }

    if (fullSyncOk)
    {
      fullSyncDone = true;
      changed &= ~DISPLAY_CHANGE_FULL_FILTER_SYNC;
      changed &= ~perFilterMask;
      if (updateDisplayRegister(REG_FILTER_SELECT, htonl(selectedFilterBand)))
      {
        changed &= ~DISPLAY_CHANGE_FILTER_SELECT;
      }
      else
      {
        changed |= DISPLAY_CHANGE_FILTER_SELECT;
      }
    }
    else
    {
      displayChangeBitmap = changed;
      Serial.printf("Display update deferred, pending bitmap: 0x%04X\n", changed);
      return;
    }
  }
  else if (changed & DISPLAY_CHANGE_FULL_FILTER_SYNC)
  {
    // Full sync already done this session, skip it and clear the flag
    changed &= ~DISPLAY_CHANGE_FULL_FILTER_SYNC;
  }

  if (changed & DISPLAY_CHANGE_FILTER_SELECT)
  {
    if (updateDisplayRegister(REG_FILTER_SELECT, htonl(selectedFilterBand)))
    {
      changed &= ~DISPLAY_CHANGE_FILTER_SELECT;
    }
  }

  if (changed & DISPLAY_CHANGE_DISPLAY_MODE)
  {
    if (updateDisplayRegister(REG_DISPLAY_MODE, htonl(displayMode)))
    {
      changed &= ~DISPLAY_CHANGE_DISPLAY_MODE;
    }
  }

  if (changed & DISPLAY_CHANGE_FILTER_TYPE)
  {
    if (updateDisplayRegister(REG_FILTER_TYPE, htonl(filterSettings[selectedFilterBand].type)))
    {
      changed &= ~DISPLAY_CHANGE_FILTER_TYPE;
    }
  }

  if (changed & DISPLAY_CHANGE_FILTER_COEFFS)
  {
    const sample_t *coeffs = filter.getCoefficients(selectedFilterBand);
    bool coeffsOk = true;
    coeffsOk = coeffsOk && updateDisplayRegister(REG_FILTER_COEFF0, htonf(coeffs[0]));
    coeffsOk = coeffsOk && updateDisplayRegister(REG_FILTER_COEFF1, htonf(coeffs[1]));
    coeffsOk = coeffsOk && updateDisplayRegister(REG_FILTER_COEFF2, htonf(coeffs[2]));
    coeffsOk = coeffsOk && updateDisplayRegister(REG_FILTER_COEFF3, htonf(-coeffs[3]));
    coeffsOk = coeffsOk && updateDisplayRegister(REG_FILTER_COEFF4, htonf(-coeffs[4]));

    if (coeffsOk)
    {
      changed &= ~DISPLAY_CHANGE_FILTER_COEFFS;
    }
  }

  if (changed & DISPLAY_CHANGE_FILTER_FREQ)
  {
    if (updateDisplayRegister(REG_FILTER_FREQ, htonf(filterSettings[selectedFilterBand].frequency)))
    {
      changed &= ~DISPLAY_CHANGE_FILTER_FREQ;
    }
  }

  if (changed & DISPLAY_CHANGE_FILTER_Q)
  {
    if (updateDisplayRegister(REG_FILTER_Q, htonf(filterSettings[selectedFilterBand].Q)))
    {
      changed &= ~DISPLAY_CHANGE_FILTER_Q;
    }
  }

  if (changed & DISPLAY_CHANGE_FILTER_GAIN)
  {
    if (updateDisplayRegister(REG_FILTER_GAIN, htonf(filterSettings[selectedFilterBand].gain)))
    {
      changed &= ~DISPLAY_CHANGE_FILTER_GAIN;
    }
  }

  if (changed & DISPLAY_CHANGE_IN_GAIN)
  {
    if (updateDisplayRegister(REG_IN_GAIN, htonf(masterGain)))
    {
      changed &= ~DISPLAY_CHANGE_IN_GAIN;
    }
  }

  if (changed & DISPLAY_CHANGE_OUT_GAIN)
  {
    if (updateDisplayRegister(REG_OUT_GAIN, htonf(volume)))
    {
      changed &= ~DISPLAY_CHANGE_OUT_GAIN;
    }
  }

  if (changed == 0)
  {
    updateSampleRate();
    Serial.printf("Display change bitmap: 0x%04X\n", before);
    commit();
    displayChangeBitmap = 0;
    Serial.println("Display update complete.");
    return;
  }

  displayChangeBitmap = changed;
  Serial.printf("Display update deferred, pending bitmap: 0x%04X\n", changed);
}
