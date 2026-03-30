#pragma once

#include <stddef.h>
#include <stdint.h>

class TFT_eSPI;

struct UiData {
  uint8_t filterType;
  uint8_t filterIndex;
  uint32_t displayMode;
  uint32_t uiMode;
  uint16_t vuLeft;
  uint16_t vuRight;
  uint8_t fftLeft[16];

  uint32_t sampleRate;
  float inputGain;
  float filterGain;
  float q;
  float outputGain;
  float frequency;
};

void ui_lvgl_init(TFT_eSPI &tft);
void ui_lvgl_task();
void ui_lvgl_update(const UiData &data, const float *selectedResponse, const float *combinedResponse, size_t pointCount);
void ui_lvgl_update_vu(uint16_t leftLevel, uint16_t rightLevel);
