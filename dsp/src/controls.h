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
  sample_t *getCoefficients() const { return filter.getCoefficients(selectedBand); }

  void setFrequency(float freq);
  void setQ(float qValue);
  void setGain(float gainValue);
  void setMasterGain(float masterGainValue);
  void setVolume(float volumeValue);
  void setDisplayMode(int mode);
  void setFilterType(int type);
  void setSelectedBand(int band);
  void setCurrentFilterSettings(FilterSettings settings);
  void setFilterSettings(int band, FilterSettings settings);

  bool isDirty() const { return dirty; }
  bool isBandChanged() const { return bandChanged; }
  bool isFilterChanged() const { return filterChanged; }

  void reset();

private:
  float masterGain;
  float volume;
  int displayMode;
  int selectedBand;
  bool dirty = false;
  bool bandChanged = false;
  bool filterChanged = false;
  FilterSettings filterSettings[FILTER_BANDS];
  AudioFilterBiquadFloat &filter;
};
