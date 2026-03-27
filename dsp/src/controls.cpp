#include "controls.h"
#include <cstring>

void ControlValues::setFrequency(float freq)
{
if(getCurrentFilter().frequency != freq) {
    dirty = filterChanged = true;
  }
  getCurrentFilter().frequency = freq;
}

void ControlValues::setQ(float qValue)
{
  if(getCurrentFilter().Q != qValue) {
    dirty = filterChanged = true;
  }
  getCurrentFilter().Q = qValue;
}

void ControlValues::setGain(float gainValue)
{
  if(getCurrentFilter().gain != gainValue) {
    dirty = filterChanged = true;
  }
  getCurrentFilter().gain = gainValue;
  filterChanged = true;
}

void ControlValues::setMasterGain(float masterGainValue)
{
  if(masterGain != masterGainValue) {
    dirty = true;
  }
  masterGain = masterGainValue;
}

void ControlValues::setVolume(float volumeValue)
{
  if(volume != volumeValue) {
    dirty = true;
  }
  volume = volumeValue;
}

void ControlValues::setDisplayMode(int mode)
{
  if(dirty = displayMode != mode) {
    dirty = true;
  }
  displayMode = mode;
}

void ControlValues::setFilterType(int type)
{
  if(getCurrentFilter().type != type) {
    filterChanged = dirty = true;
  }
  getCurrentFilter().type = type;
}

void ControlValues::setSelectedBand(int band)
{
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

void ControlValues::reset()
{
  dirty = false;
  bandChanged = false;
  filterChanged = false;
}
