#include "controls.h"
#include "../../common/constants.h"
#include <cmath>
#include <cstring>

namespace
{
constexpr float kMinFrequency = 10.0f;
constexpr float kMaxFrequency = 20000.0f;
constexpr float kDefaultFrequency = 1000.0f;
constexpr float kDefaultQ = 0.707f;
constexpr float kDefaultGain = 0.0f;
constexpr float kDefaultMasterGain = 0.0f;
constexpr float kDefaultVolume = 0.0f;
constexpr float kMinMasterGain = -20.0f;
constexpr float kMaxMasterGain = 20.0f;
constexpr float kMinVolume = -127.0f;
constexpr float kMaxVolume = 0.0f;

float clampFloat(float value, float minValue, float maxValue, float fallback)
{
  if (!std::isfinite(value))
  {
    return fallback;
  }
  if (value < minValue)
  {
    return minValue;
  }
  if (value > maxValue)
  {
    return maxValue;
  }
  return value;
}

FilterSettings sanitizeFilterSettings(FilterSettings settings)
{
  if (settings.type >= NUM_FILTER_TYPES)
  {
    settings.type = BYPASS;
  }
  settings.frequency = clampFloat(settings.frequency, kMinFrequency, kMaxFrequency, kDefaultFrequency);
  settings.Q = clampFloat(settings.Q, 0.1f, 5.0f, kDefaultQ);
  settings.gain = clampFloat(settings.gain, -15.0f, 15.0f, kDefaultGain);
  return settings;
}

int clampInt(int value, int minValue, int maxValue, int fallback)
{
  if (value < minValue || value > maxValue)
  {
    return fallback;
  }
  return value;
}
}

void ControlValues::setFrequency(float freq)
{
  freq = clampFloat(freq, kMinFrequency, kMaxFrequency, kDefaultFrequency);
  if(getCurrentFilter().frequency != freq) {
    dirty = filterChanged = true;
  }
  getCurrentFilter().frequency = freq;
}

void ControlValues::setQ(float qValue)
{
  qValue = clampFloat(qValue, 0.1f, 5.0f, kDefaultQ);
  if(getCurrentFilter().Q != qValue) {
    dirty = filterChanged = true;
  }
  getCurrentFilter().Q = qValue;
}

void ControlValues::setGain(float gainValue)
{
  gainValue = clampFloat(gainValue, -15.0f, 15.0f, kDefaultGain);
  if(getCurrentFilter().gain != gainValue) {
    dirty = filterChanged = true;
  }
  getCurrentFilter().gain = gainValue;
  filterChanged = true;
}

void ControlValues::setMasterGain(float masterGainValue)
{
  masterGainValue = clampFloat(masterGainValue, kMinMasterGain, kMaxMasterGain, kDefaultMasterGain);
  if(masterGain != masterGainValue) {
    dirty = true;
  }
  masterGain = masterGainValue;
}

void ControlValues::setVolume(float volumeValue)
{
  volumeValue = clampFloat(volumeValue, kMinVolume, kMaxVolume, kDefaultVolume);
  if(volume != volumeValue) {
    dirty = true;
  }
  volume = volumeValue;
}

void ControlValues::setDisplayMode(int mode)
{
  mode = clampInt(mode, 0, NUM_DISPLAY_MODES - 1, 0);
  if(displayMode != mode) {
    dirty = true;
  }
  displayMode = mode;
}

void ControlValues::setFilterType(int type)
{
  type = clampInt(type, 0, NUM_FILTER_TYPES - 1, BYPASS);
  if(getCurrentFilter().type != type) {
    filterChanged = dirty = true;
  }
  getCurrentFilter().type = type;
}

void ControlValues::setSelectedBand(int band)
{
  band = clampInt(band, 0, FILTER_BANDS - 1, 0);
  if(selectedBand != band) {
    filterChanged = bandChanged = dirty = true;
  }
  selectedBand = band;
}

void ControlValues::setCurrentFilterSettings(FilterSettings settings)
{
  setFilterSettings(selectedBand, settings);
}

void ControlValues::setFilterSettings(int band, FilterSettings settings)
{
  if (band < 0 || band >= FILTER_BANDS)
  {
    return;
  }
  settings = sanitizeFilterSettings(settings);
  if (memcmp(&filterSettings[band], &settings, sizeof(FilterSettings)) != 0) {
    dirty = filterChanged = true;
  }
  filterSettings[band] = settings;
}

void ControlValues::cycleSelectedBand() {
  setSelectedBand((getSelectedBand() + 1) % FILTER_BANDS);
}   

void ControlValues::cycleFilterType() {
  setFilterType((getFilterType() + 1) % NUM_FILTER_TYPES);
}   

void ControlValues::cycleDisplayMode() {
  setDisplayMode((getDisplayMode() + 1) % NUM_DISPLAY_MODES);
}

void ControlValues::cycleUIMode() {
  setUIMode((getUIMode() + 1) % NUM_UI_MODES);
}   

void ControlValues::setUIMode(int mode) {
  mode = clampInt(mode, 0, NUM_UI_MODES - 1, UI_MODE_DETAILED);
  if(uiMode != mode) {
    dirty = true;
  }
  uiMode = mode;
}

void ControlValues::reset()
{
  dirty = false;
  bandChanged = false;
  filterChanged = false;
}
