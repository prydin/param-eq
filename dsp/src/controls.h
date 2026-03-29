#pragma once
#include <Arduino.h>
#include "../../common/filter.h"
#include "audio_pipeline/filter_biquad_f.h"

class ControlValues
{
public:
  ControlValues(AudioFilterBiquadFloat &filter) : filter(filter) {}
  FilterSettings &getCurrentFilter() { return filterSettings[selectedBand]; }
  FilterSettings &getFilter(int band) { return filterSettings[band]; }
  int getFilterType() { return getCurrentFilter().type; }
  float getFrequency() { return getCurrentFilter().frequency; }
  float getQ() { return getCurrentFilter().Q; }
  float getGain() { return getCurrentFilter().gain; }
  float getMasterGain() { return masterGain; }
  float getVolume() { return volume; }
  int getDisplayMode() { return displayMode; }
  int getSelectedBand() { return selectedBand; }
  int getUIMode() { return uiMode; }
  const sample_t *getCoefficients() const { return filter.getCoefficients(selectedBand); }

  void setFrequency(float freq);
  void setQ(float qValue);
  void setGain(float gainValue);
  void setMasterGain(float masterGainValue);
  void setVolume(float volumeValue);
  void setDisplayMode(int mode);
  void setFilterType(int type);
  void setSelectedBand(int band);
  void setUIMode(int mode);
  void cycleUIMode();
  void cycleSelectedBand();
  void cycleFilterType();
  void cycleDisplayMode();
  void setCurrentFilterSettings(FilterSettings settings);
  void setFilterSettings(int band, FilterSettings settings);

  bool isDirty() const { return dirty; }
  bool isBandChanged() const { return bandChanged; }
  bool isFilterChanged() const { return filterChanged; }

  void reset();

private:
  float masterGain = 0.0f;
  float volume = 0.0f;
  int displayMode = 0;
  int selectedBand = 0;
  int uiMode = 0;
  bool dirty = false;
  bool bandChanged = false;
  bool filterChanged = false;
  FilterSettings filterSettings[FILTER_BANDS];
  AudioFilterBiquadFloat &filter;
};
