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
#include <Wire.h>

#include "../../common/filter.h"
#include "../../common/constants.h"
#include "audio_pipeline/filter_biquad_f.h"

class Display
{
public:
  static constexpr uint16_t DISPLAY_CHANGE_FILTER_SELECT = 1u << 0;
  static constexpr uint16_t DISPLAY_CHANGE_DISPLAY_MODE = 1u << 1;
  static constexpr uint16_t DISPLAY_CHANGE_FILTER_TYPE = 1u << 2;
  static constexpr uint16_t DISPLAY_CHANGE_FILTER_FREQ = 1u << 3;
  static constexpr uint16_t DISPLAY_CHANGE_FILTER_Q = 1u << 4;
  static constexpr uint16_t DISPLAY_CHANGE_FILTER_GAIN = 1u << 5;
  static constexpr uint16_t DISPLAY_CHANGE_IN_GAIN = 1u << 6;
  static constexpr uint16_t DISPLAY_CHANGE_OUT_GAIN = 1u << 7;
  static constexpr uint16_t DISPLAY_CHANGE_FILTER_COEFFS = 1u << 8;
  static constexpr uint16_t DISPLAY_CHANGE_ALL = DISPLAY_CHANGE_FILTER_SELECT |
                                                 DISPLAY_CHANGE_DISPLAY_MODE |
                                                 DISPLAY_CHANGE_FILTER_TYPE |
                                                 DISPLAY_CHANGE_FILTER_FREQ |
                                                 DISPLAY_CHANGE_FILTER_Q |
                                                 DISPLAY_CHANGE_FILTER_GAIN |
                                                 DISPLAY_CHANGE_IN_GAIN |
                                                 DISPLAY_CHANGE_OUT_GAIN |
                                                 DISPLAY_CHANGE_FILTER_COEFFS;

  Display(TwoWire &wire,
          AudioFilterBiquadFloat &filter,
          FilterSettings *filterSettings,
          int &selectedFilterBand,
          int &displayMode,
          float &masterGain,
          float &volume,
          uint16_t &displayChangeBitmap);

  void updateDisplayRegister(uint8_t reg, uint32_t value);
  void updateDisplay();
  void updateSampleRate();
  void commit();

private:
  TwoWire &wire;
  AudioFilterBiquadFloat &filter;
  FilterSettings *filterSettings;
  int &selectedFilterBand;
  int &displayMode;
  float &masterGain;
  float &volume;
  uint16_t &displayChangeBitmap;
};
