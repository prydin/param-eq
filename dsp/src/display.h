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

#include <Arduino.h>

#include "controls.h"
#include "../../common/filter.h"
#include "../../common/constants.h"
#include "audio_pipeline/filter_biquad_f.h"

class Display
{
public:
  Display() {}

  void setFilterSettings(FilterSettings *settings = nullptr, const sample_t *coeffs = nullptr, bool force = false);
  void setFilterBand(int band, bool force = false);
  void setDisplayMode(int mode, bool force = false);
  void setMasterGain(float gain, bool force = false);
  void setVolume(float volume, bool force = false);
  void setSampleRate(int sampleRate, bool force = false);
  void setUIMode(int uiMode, bool force = false);
  void setVUMeterValue(float left, float right, bool force = false);
  void setFFTValues(const float *values, bool force = false);
  void pushInitialSettings(ControlValues &controlValues);
  void update(ControlValues &controlValues);
  void commit();

private:
  static constexpr size_t DISPLAY_REGISTER_COUNT = 256;

  void sendRegister(uint8_t reg, uint32_t value, bool force = false);
  void updateFilterBand(int band, bool force = false);

  uint32_t registerCache[DISPLAY_REGISTER_COUNT] = {};
};

