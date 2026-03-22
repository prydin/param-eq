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
#include <Arduino.h>
#include "../../common/filter.h"

// A a struct of all user settings that need to be persisted
struct PersistedSettings
{
    uint32_t magic;                            // Magic number for validation
    uint32_t version;                          // Version number for future compatibility
    FilterSettings filterSettings[FILTER_BANDS]; // Settings for low, mid, high bands
    uint8_t selectedFilterBand;                // Currently selected filter band
    uint8_t displayMode;                       // Current display mode
    uint16_t reserved;                         // Explicit padding for stable EEPROM layout
    float masterGain;                          // Current input/master gain in dB
    float volume;                              // Current volume
    uint32_t checksum;                         // Checksum for data integrity
};

bool saveSettings(const PersistedSettings settings);
bool loadSettings(PersistedSettings& settings);
void clearSettings();   