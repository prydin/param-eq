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
/**
 * @file main.cpp
 * @brief Teensy-based parametric equalizer with rotary encoder controls and I2S audio output.
 *
 * This file implements a real-time audio parametric equalizer system using the
 * Teensy Audio Library.
 * It provides three-band equalization (low, mid, high) with selectable filter types (low shelf,
 * high shelf, peaking EQ). User controls include rotary encoders for frequency, gain, and Q
 * adjustment, along with pushbuttons for filter band and type selection.
 *
 * The system generates a test waveform, processes it through stereo biquad filters, and outputs
 * the result via I2S. Filter coefficients and parameters are transmitted to an external display
 * via I2C communication.
 *
 * @note Requires Teensy Audio Library, OneButton, ErriezCRC32, and AcceleratedEncoder libraries.
 * @note I2C display address is hardcoded to 0xb1ce.
 * @note Audio processing uses floating-point biquad filters for precise frequency response control.
 */
#include <Teensy4i2s.h>
#include <Wire.h>
#include <SD.h>
#include <SerialFlash.h>
#include <OneButton.h>
#include <ErriezCRC32.h>
#include <stdlib.h>
#include "../../common/filter.h"
#include "../../common/constants.h"
#include "audio_pipeline/filter_biquad_f.h"
#include "audio_pipeline/audio_square_wave.h"
#include "audio_pipeline/audio_gain.h"
#include "audio_pipeline/audio_rms.h"
#include "audio_pipeline/audio_peak.h"
#include "audio_pipeline/audio_controller.h"
#include "audio_pipeline/audio_spectrum_td.h"
#include "AcceleratedEncoder.h"
#include "netconv.h"
#include "persistence.h"
#include "display.h"

//#define TESTMODE 1

#ifdef TESTMODE
#include "audio_pipeline/test_tone.h"
#endif
#include <ES9039Q2M.h>

// Filter control endpoints and steps
#define MIN_GAIN_DB -15.0f
#define MAX_GAIN_DB 15.0f
#define GAIN_STEP_DB 0.1f
#define MIN_Q 0.1f
#define MAX_Q 5.0f
#define Q_STEP 0.1f
#define MIN_FREQUENCY 10.0f
#define LOG_MIN_FREQUENCY (log10f(MIN_FREQUENCY))
#define MAX_FREQUENCY 20000.0f
#define LOG_MAX_FREQUENCY (log10f(MAX_FREQUENCY))
#define FREQ_RESOLUTION 400.0f
#define LOG_FREQ_STEP ((LOG_MAX_FREQUENCY - LOG_MIN_FREQUENCY) / FREQ_RESOLUTION)
#define MASTER_GAIN_STEP_DB 0.1f
#define MASTER_GAIN_MIN -20.0f
#define MASTER_GAIN_MAX 20.0f
#define VOLUME_STEP_DB 1.0f
#define VOLUME_MIN -127.0f
#define VOLUME_MAX 0.0f

// Frequency conversion macros
#define FREQ_TO_INDEX(freq) (map(log10(freq),                       \
                                 LOG_MIN_FREQUENCY,                 \
                                 LOG_MAX_FREQUENCY,                 \
                                 LOG_MIN_FREQUENCY / LOG_FREQ_STEP, \
                                 LOG_MAX_FREQUENCY / LOG_FREQ_STEP))

// Rotary encoder pins
#define FC_PIN_A 33
#define FC_PIN_B 34
#define FILTER_TYPE_PIN 35
#define GAIN_PIN_A 36
#define GAIN_PIN_B 37
#define FILTER_INDEX_PIN 38
#define Q_PIN_A 39
#define Q_PIN_B 40
#define FILTER_MODE_PIN 41
#define INPUT_GAIN_PIN_A 2
#define INPUT_GAIN_PIN_B 3
#define VOLUME_PIN_A 11
#define VOLUME_PIN_B 12
#define UI_MODE_PIN 1

// Settings save interval
#define SAVE_INTERVAL_MS 5000

// Dely before clearing clip indication
#define CLIP_CLEAR_DELAY_MS 100

// DAC initialization retries
#define DAC_INIT_RETRIES 3
#define DAC_INIT_DELAY_MS 200
#define DAC_CHECK_PERIOD 5000

// DAC instance
ES9039Q2M dac;

// Rotary encoders
AcceleratedEncoder fcSelector(FC_PIN_A, FC_PIN_B);
AcceleratedEncoder gainSelector(GAIN_PIN_A, GAIN_PIN_B);
AcceleratedEncoder qSelector(Q_PIN_A, Q_PIN_B);
AcceleratedEncoder masterGainSelector(INPUT_GAIN_PIN_A, INPUT_GAIN_PIN_B);
AcceleratedEncoder volumeSelector(VOLUME_PIN_A, VOLUME_PIN_B);

// Pushbuttons
OneButton filterSelectButton(FILTER_INDEX_PIN, true);
OneButton filterTypeSelectButton(FILTER_TYPE_PIN, true);
OneButton displayModeButton(FILTER_MODE_PIN, true);
OneButton uiModeButton(UI_MODE_PIN, true);

AudioSquareWave waveform;
#ifdef TESTMODE
TestTone testTone(100, 48000);
#endif

// The filters
AudioFilterBiquadFloat filter;

AudioGain gain;
AudioRMS rmsMeter;
AudioPeak peakMeter;
AudioSpectrumTD spectrum;

ControlValues controlValues(filter);
Display display;

// Last time settings were saved to EEPROM
unsigned long lastSaveTime = 0;
bool saveNeeded = false;

// DAC state
bool dacRunning = false;

// Clipping state
typedef enum ClipState
{
  NO_CLIP,
  CLIP_TRIGGERED,
  CLIP_ACTIVE
} ClipState;

ClipState clipState = NO_CLIP;
uint32_t clipTimestamp = 0;

bool checkDAC()
{
  return dac.getChipID() != 0;
}

/**
 * @brief Initializes the ES9039Q2M DAC with default settings.
 *
 * This function performs a reset of the DAC, applies initial configuration settings,
 * and sets the initial volume level based on the global `volume` variable.
 *
 * @note The DAC is reset by writing 0x82 to register 0x00, followed by a delay of 500 ms.
 * @note Initial configuration includes setting system config, volume, and other necessary
 *       registers for proper operation.
 * @note The volume is set in decibels (dB) using the setVolumeDB() method of the DAC class.
 */
void initDAC()
{
  // Reset the DAC
  for (int i = 0; i < DAC_INIT_RETRIES; i++)
  {
    dac.writeRegister8(0x00, 0x82);
    delay(DAC_INIT_DELAY_MS);
    if (checkDAC() != 0)
    {
      dacRunning = true;
      break; // Exit loop if DAC responds
    }
  }
  if (!dacRunning)
  {
    Serial.println("Failed to initialize DAC after multiple attempts.");
    return;
  }

  // Initial settings
  dac.writeRegister8(0x00, 0x02);
  // Configure DAC for externally clocked PCM (I2S slave pipeline).

  // DO NOT CHANGE!
  dac.writeRegister8(0x00, 0x02);
  dac.writeRegister8(0x01, 0xB1);
  dac.writeRegister8(0x39, 0x41); // 41
  dac.writeRegister8(0x56, 0x00);
  dac.writeRegister8(0x7B, 0x00);
  // END DO NOT CHANGE!
  dac.setVolumeDB(controlValues.getVolume());
}

/**
 * @brief Updates the filter coefficients for the selected filter band.
 *
 * This function configures both left and right channel filters based on the
 * filter type and parameters stored in the filterSettings array for the
 * specified band.
 *
 * @param selected The index of the filter band to update
 *
 * @note The function applies the same filter settings to both left and right
 *       channels to maintain stereo coherence.
 * @note Supported filter types: LOWSHELF, HIGHSHELF, PEAKINGEQ, and BYPASS
 */
void updateFilter(int selected)
{
  FilterSettings settings = controlValues.getFilter(selected);
  if (!isfinite(settings.frequency) || !isfinite(settings.Q) || !isfinite(settings.gain))
  {
    filter.bypass(selected);
    return;
  }
  switch (settings.type)
  {
  case LOWSHELF:
    filter.setLowShelf(selected, settings.frequency, AudioController::getSampleRate(), settings.gain, settings.Q);
    break;
  case HIGHSHELF:
    filter.setHighShelf(selected, settings.frequency, AudioController::getSampleRate(), settings.gain, settings.Q);
    break;
  case PEAKINGEQ:
    filter.setPeakingEQ(selected, settings.frequency, AudioController::getSampleRate(), settings.Q, settings.gain);
    break;
  case BYPASS:
    filter.bypass(selected);
  }
}

/**
 * @brief Updates all filter bands by applying the current settings.
 *
 * This function iterates through all filter bands defined by FILTER_BANDS
 * and calls updateFilter() for each band to ensure that the filter
 * coefficients are up to date based on the current settings.
 */
void updateAllFilters()
{
  for (int i = 0; i < FILTER_BANDS; i++)
  {
    updateFilter(i);
  }
}

void clipDetected(bool clipped)
{
  if (clipped)
  {
    if (clipState == NO_CLIP)
    {
      clipState = CLIP_TRIGGERED;
    }
  }
}

void initDisplay()
{
  display.setMasterGain(controlValues.getMasterGain());
  display.setVolume(controlValues.getVolume());
  display.setUIMode(controlValues.getUIMode());
  updateAllFilters();
  display.commit();
}

/**
 * @brief Initializes the audio processing system, UI controls, and hardware interfaces.
 *
 * This function sets up the following components:
 * - Serial communication at 115000 baud
 * - Audio memory allocation (12 blocks)
 * - Waveform generator configured as square wave at 980 Hz with 0.3 amplitude
 * - Rotary encoder pins for frequency control (FC), gain, Q factor, and filter index
 * - Filter type selection button with pull-up resistor
 * - Frequency selector with range 2-2000 Hz, initialized at 100 (1 kHz)
 * - Gain selector initialized to PEAKINGEQ mode
 * - Q factor selector initialized to 707 (Q=0.707)
 * - Filter type button callback to cycle through 3 filter types
 * - I2C communication via Wire interface
 * - Additional analog inputs (A10, A11, A12) and digital pin 10 as chip select
 * - Audio mixer gains set to 0.5 for both left and right channels
 */
void setup(void)
{
  Serial.begin(115000);

  gain.setTimingName("gain");
  filter.setTimingName("filter_biquad");
  rmsMeter.setTimingName("rms");
  peakMeter.setTimingName("peak");
  spectrum.setTimingName("spectrum_td");
  AudioController::getInstance()->setTimingName("audio_controller");

  // Build audio pipeline
  // Source -> Gain -> Filter -> I2S
  gain.addReceiver(&filter);
  filter.addReceiver(&rmsMeter);
  filter.addReceiver(&peakMeter);
  filter.addReceiver(&spectrum);
  filter.addReceiver(AudioController::getInstance());
  gain.setGain(1.0f);
  spectrum.configure(FFT_DISPLAY_BINS, 48000, 20.0, 20000.0,
                     AudioSpectrumTD::Spacing::Logarithmic,
                     AudioSpectrumTD::Detector::Peak,
                     1.0, 20.0 * log10(5.0));  // linear gain x5
  spectrum.setEnabled(true);

#ifdef TESTMODE
  testTone.addReceiver(&gain);
  AudioController::getInstance()->addReceiver(&testTone);
#else
  AudioController::getInstance()->addReceiver(&gain);
#endif
  AudioController::getInstance()->setClipDetector(clipDetected);

  // Make sure pins 5 and 6 are in input mode (due to a PCB design issue)
  pinMode(5, INPUT);
  pinMode(6, INPUT);

  // Initialize the DAC
  dac.begin();
  delay(500); // Wait for DAC to stabilize
  initDAC();

  // need to wait a bit before configuring codec,
  // otherwise something weird happens and there's no output...
  delay(100);

#ifdef TESTMODE
  InitI2s(false);
  AudioController::getInstance()->setSampleRate(96000);

  // testTone.begin();
#else
  InitI2s(true);
#endif
  // waveform.setAmplitude(0.5f);
  // waveform.setFrequency(980.0f); // 980 Hz test tone
  // waveform.addReceiver(&gain);

  // Initialize UI
  // Set all the rotary encoder pints to input mode
  pinMode(FC_PIN_A, INPUT);
  pinMode(FC_PIN_B, INPUT);
  pinMode(FILTER_TYPE_PIN, INPUT_PULLUP);
  pinMode(GAIN_PIN_A, INPUT);
  pinMode(GAIN_PIN_B, INPUT);
  pinMode(FILTER_INDEX_PIN, INPUT);
  pinMode(Q_PIN_A, INPUT);
  pinMode(Q_PIN_B, INPUT);
  pinMode(INPUT_GAIN_PIN_A, INPUT_PULLUP);
  pinMode(INPUT_GAIN_PIN_B, INPUT_PULLUP);

  // Initialize clip LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Set up pushbuttons
  filterTypeSelectButton.attachPress([]()
                                     { controlValues.cycleFilterType(); });
  filterSelectButton.attachPress([]()
                                 { controlValues.cycleSelectedBand(); });
  displayModeButton.attachPress([]()
                                { controlValues.cycleDisplayMode(); });
  uiModeButton.attachPress([]()
                           { controlValues.cycleUIMode(); });

  // Initialize S2C
  Wire.begin();

  // Set up Q control on A10
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  pinMode(A12, INPUT);
  pinMode(10, OUTPUT);

  // Reset last save time
  lastSaveTime = millis();

  // Load persisted settings
  PersistedSettings persistedSettings;
  if (loadSettings(persistedSettings))
  {
    for (int i = 0; i < FILTER_BANDS; i++)
    {
      controlValues.setFilterSettings(i, persistedSettings.filterSettings[i]);
    }
    controlValues.setSelectedBand(persistedSettings.selectedFilterBand);
    controlValues.setDisplayMode(persistedSettings.displayMode);
    controlValues.setMasterGain(persistedSettings.masterGain);
    controlValues.setVolume(persistedSettings.volume);
    gain.setGain(powf(10.0f, controlValues.getMasterGain() / 20.0f));
    dac.setVolumeDB(controlValues.getVolume());
    Serial.println("Loaded persisted settings from EEPROM.");
  }
  else
  {
    // Initialize default settings
    Serial.println("No persisted settings found, using defaults.");
    for (int i = 0; i < FILTER_BANDS; i++)
    {
      FilterSettings settings;
      settings.frequency = 1000.0f; // 1 kHz
      settings.Q = 0.707f;          // Q=0.707
      settings.gain = 0.0f;         // 0 dB
      settings.type = BYPASS;       // Bypass
      controlValues.setFilterSettings(i, settings);
    }
  }

  // Set initial positions — setEndpoints MUST come before setAcceleratedPosition so
  // the clamping inside setAcceleratedPosition uses the correct range.
  FilterSettings settings = controlValues.getCurrentFilter();
  Serial.printf("Selected filter: %d, Initial frequency: %f Hz, Q: %f, Gain: %f dB, Index: %d\n",
                controlValues.getSelectedBand(), settings.frequency, settings.Q, settings.gain, FREQ_TO_INDEX(settings.frequency));
  fcSelector.setEndpoints(log10(MIN_FREQUENCY) / LOG_FREQ_STEP,
                          log10(MAX_FREQUENCY) / LOG_FREQ_STEP);
  fcSelector.setAcceleratedPosition(FREQ_TO_INDEX(settings.frequency));
  gainSelector.setEndpoints(MIN_GAIN_DB / GAIN_STEP_DB, MAX_GAIN_DB / GAIN_STEP_DB);
  gainSelector.setAcceleratedPosition(settings.gain / GAIN_STEP_DB);
  qSelector.setEndpoints(MIN_Q / Q_STEP, MAX_Q / Q_STEP);
  qSelector.setAcceleratedPosition(settings.Q / Q_STEP);
  masterGainSelector.setEndpoints(
      MASTER_GAIN_MIN / MASTER_GAIN_STEP_DB,
      MASTER_GAIN_MAX / MASTER_GAIN_STEP_DB); // -20 dB to 20 dB
  masterGainSelector.setAcceleratedPosition(controlValues.getMasterGain() / MASTER_GAIN_STEP_DB);
  volumeSelector.setEndpoints(VOLUME_MIN / VOLUME_STEP_DB, VOLUME_MAX / VOLUME_STEP_DB);
  volumeSelector.setAcceleratedPosition(controlValues.getVolume() / VOLUME_STEP_DB);

  // Initial display update
  updateAllFilters();
  delay(1000); // Wait for display to be fully initialized
  initDisplay();
}

/**
 * This function is called from the loop and blinks the buiitin LED once a second.
 */
void blinkLED()
{
  static time_t nextToggle = 0;
  static bool state = false;
  time_t currentTime = millis();
  if (currentTime >= nextToggle)
  {
    digitalWrite(LED_BUILTIN, state ? HIGH : LOW);
    Serial.printf("Flipped state to %d\n", state);
    nextToggle = currentTime + 500; // Toggle every second
  }
}

uint32_t getCurrentSampleRate(bool *isChanged = nullptr)
{
  static uint32_t lastRate = 44100; // Default to 44.1 kHz until we get a reading from AudioController
  uint32_t newRate = AudioController::getStandardizedSampleRate();
  if (isChanged)
  {
    *isChanged = (newRate != lastRate);
  }
  if (AudioController::isSampleRateStable())
  {
    lastRate = newRate;
  }
  return newRate;
}

void checkDACConnection(time_t currentTime, time_t &lastDACCheck)
{
  if (currentTime - lastDACCheck <= DAC_CHECK_PERIOD)
  {
    return;
  }

  if (!checkDAC())
  {
    dacRunning = false;
    Serial.println("Lost contact with DAC!.");
  }
  else if (!dacRunning)
  {
    Serial.println("DAC came back online. Re-initializing.");
    initDAC();
    digitalWrite(LED_BUILTIN, LOW);
  }

  lastDACCheck = currentTime;
}

void updateSampleRateStatus(uint32_t &lastSampleRate, bool &lastSampleRateStable)
{
  uint32_t currentSampleRate = AudioController::getStandardizedSampleRate();
  bool currentSampleRateStable = AudioController::isSampleRateStable();
  if (currentSampleRate == lastSampleRate &&
      currentSampleRateStable == lastSampleRateStable)
  {
    return;
  }

  // Retune only when stable to avoid invalid coefficients during clock transients.
  if (currentSampleRateStable)
  {
    updateAllFilters();
    spectrum.configure(FFT_DISPLAY_BINS, currentSampleRate, 20.0, 20000.0,
                       AudioSpectrumTD::Spacing::Logarithmic,
                       AudioSpectrumTD::Detector::RMS,
                       1.0, 20.0 * log10(5.0));  // linear gain x5
  }

  lastSampleRate = currentSampleRate;
  lastSampleRateStable = currentSampleRateStable;
  display.setSampleRate(currentSampleRate, true);
  display.commit();
  Serial.printf("Sample rate changed: %u Hz (stable=%s)\n",
                currentSampleRate,
                currentSampleRateStable ? "true" : "false");
}

void printStatusIfNeeded(time_t currentTime, time_t &nextStatusPrint)
{
  static uint32_t lastProcessCount = 0;

  if (currentTime < nextStatusPrint)
  {
    return;
  }

  Serial.println("----- Status -----");
  float avgCpuLoadPct = Timers::GetCpuLoad() * 100.0f;
  float peakCpuLoadPct = 0.0f;
  float avgPeriod = Timers::GetAvgPeriod();
  float avgProcUs = Timers::GetAvg(Timers::TIMER_TOTAL);
  float peakProcUs = Timers::GetPeak(Timers::TIMER_TOTAL);
  const float cpuHz = 600000000.0f; // Teensy 4.x nominal core clock
  float avgCycles = avgProcUs * (cpuHz / 1000000.0f);
  float peakCycles = peakProcUs * (cpuHz / 1000000.0f);
  float budgetCycles = avgPeriod * (cpuHz / 1000000.0f);
  float avgHeadroomPct = 0.0f;
  if (avgPeriod > 0.0f)
  {
    peakCpuLoadPct = (Timers::GetPeak(Timers::TIMER_TOTAL) / avgPeriod) * 100.0f;
    avgHeadroomPct = ((avgPeriod - avgProcUs) / avgPeriod) * 100.0f;
  }
  Serial.printf(
      "CPU Load avg: %.2f%%, peak: %.2f%%, Sample rate: %u Hz, stable: %s, stable intervals: %u, inputAlign: %s (%u-bit)\n",
      avgCpuLoadPct,
      peakCpuLoadPct,
      AudioController::getStandardizedSampleRate(),
      AudioController::isSampleRateStable() ? "true" : "false",
      AudioController::getNumStableIntervals(),
      AudioController::isInputAlignmentLocked() ? "locked" : "unlocked",
      (unsigned)AudioController::getInputShiftBits());
  Serial.printf(
      "Frame timing us (avg/peak/budget): %.2f / %.2f / %.2f, cycles (avg/peak/budget): %.0f / %.0f / %.0f, avg headroom: %.2f%%\n",
      avgProcUs,
      peakProcUs,
      avgPeriod,
      avgCycles,
      peakCycles,
      budgetCycles,
      avgHeadroomPct);
  nextStatusPrint = currentTime + 2000;
  Serial.printf("RMS Level: %f, / %f, peak: %f / %f\n", rmsMeter.getRMSLeft(), rmsMeter.getRMSRight(), peakMeter.getPeakLeft(), peakMeter.getPeakRight());
  if (controlValues.getUIMode() != UI_MODE_SIMPLE)
  {
    peakMeter.reset();
  }
  uint32_t processCount = AudioController::getProcessCount();
  Serial.printf("Audio blocks processed: %lu (+%lu)\n",
                (unsigned long)processCount,
                (unsigned long)(processCount - lastProcessCount));
  lastProcessCount = processCount;
  Serial.printf("DAC is %s\n", dac.getChipID() == 99 ? "alive" : "dead");
  AudioComponent::printTimingReport();
}

void updateClipIndicator()
{
  if (!dacRunning)
  {
    return;
  }

  if (clipState == CLIP_TRIGGERED)
  {
    clipState = CLIP_ACTIVE;
    clipTimestamp = millis();
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else if (clipState == CLIP_ACTIVE && (millis() - clipTimestamp) > CLIP_CLEAR_DELAY_MS)
  {
    clipState = NO_CLIP;
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void tickControls()
{
  fcSelector.tick();
  gainSelector.tick();
  qSelector.tick();
  filterTypeSelectButton.tick();
  filterSelectButton.tick();
  uiModeButton.tick();
  displayModeButton.tick();
  masterGainSelector.tick();
  volumeSelector.tick();
}

void syncEncoders()
{
  if (controlValues.isBandChanged())
  {
    return;
  }

  FilterSettings settings = controlValues.getCurrentFilter();
  fcSelector.setAcceleratedPosition(FREQ_TO_INDEX(settings.frequency));
  gainSelector.setAcceleratedPosition(map(settings.gain,
                                          MIN_GAIN_DB,
                                          MAX_GAIN_DB,
                                          MIN_GAIN_DB / GAIN_STEP_DB,
                                          MAX_GAIN_DB / GAIN_STEP_DB));
  qSelector.setAcceleratedPosition(map(settings.Q,
                                       MIN_Q,
                                       MAX_Q,
                                       MIN_Q / Q_STEP,
                                       MAX_Q / Q_STEP));
}

void readControlValues(ControlValues &values)
{
  auto mapf = [](float x, float inMin, float inMax, float outMin, float outMax)
  {
    if (inMax == inMin)
    {
      return outMin;
    }
    float t = (x - inMin) / (inMax - inMin);
    return outMin + t * (outMax - outMin);
  };

  float fcPos = constrain(float(fcSelector.geAcceleratedPosition()),
                          LOG_MIN_FREQUENCY / LOG_FREQ_STEP,
                          LOG_MAX_FREQUENCY / LOG_FREQ_STEP);
  float qPos = constrain(float(qSelector.geAcceleratedPosition()),
                         MIN_Q / Q_STEP,
                         MAX_Q / Q_STEP);
  float gainPos = constrain(float(gainSelector.geAcceleratedPosition()),
                            MIN_GAIN_DB / GAIN_STEP_DB,
                            MAX_GAIN_DB / GAIN_STEP_DB);
  float masterPos = constrain(float(masterGainSelector.geAcceleratedPosition()),
                              MASTER_GAIN_MIN / MASTER_GAIN_STEP_DB,
                              MASTER_GAIN_MAX / MASTER_GAIN_STEP_DB);
  float volumePos = constrain(float(volumeSelector.geAcceleratedPosition()),
                              VOLUME_MIN / VOLUME_STEP_DB,
                              VOLUME_MAX / VOLUME_STEP_DB);
  float frequencyLog = mapf(fcPos,
                            LOG_MIN_FREQUENCY / LOG_FREQ_STEP,
                            LOG_MAX_FREQUENCY / LOG_FREQ_STEP,
                            LOG_MIN_FREQUENCY,
                            LOG_MAX_FREQUENCY);

  values.setFrequency(powf(10.0f, frequencyLog));
  values.setQ(mapf(qPos,
                   MIN_Q / Q_STEP,
                   MAX_Q / Q_STEP,
                   MIN_Q,
                   MAX_Q));
  values.setGain(mapf(gainPos,
                      MIN_GAIN_DB / GAIN_STEP_DB,
                      MAX_GAIN_DB / GAIN_STEP_DB,
                      MIN_GAIN_DB,
                      MAX_GAIN_DB));
  values.setMasterGain(mapf(masterPos,
                            MASTER_GAIN_MIN / MASTER_GAIN_STEP_DB,
                            MASTER_GAIN_MAX / MASTER_GAIN_STEP_DB,
                            MASTER_GAIN_MIN,
                            MASTER_GAIN_MAX));
  values.setVolume(mapf(volumePos,
                        VOLUME_MIN / VOLUME_STEP_DB,
                        VOLUME_MAX / VOLUME_STEP_DB,
                        VOLUME_MIN,
                        VOLUME_MAX));
}

void saveSettingsIfNeeded(time_t currentTime)
{
  if (!saveNeeded || currentTime - lastSaveTime <= SAVE_INTERVAL_MS)
  {
    return;
  }

  PersistedSettings persistedSettings;
  for (int i = 0; i < FILTER_BANDS; i++)
  {
    persistedSettings.filterSettings[i] = controlValues.getFilter(i);
  }
  persistedSettings.selectedFilterBand = controlValues.getSelectedBand();
  persistedSettings.displayMode = controlValues.getDisplayMode();
  persistedSettings.masterGain = controlValues.getMasterGain();
  persistedSettings.volume = controlValues.getVolume();
  saveSettings(persistedSettings);
  lastSaveTime = currentTime;
  Serial.println("Settings saved to EEPROM.");
  saveNeeded = false;
}

/**
 * @brief Main loop function that handles rotary encoder input and updates audio filters.
 *
 * This function is called repeatedly to:
 * - Poll rotary encoders for frequency, gain, Q factor, and filter type selection
 * - Map the frequency selector position to a frequency range (0-20000 Hz)
 * - Update the left and right channel filters based on the selected filter type
 * - Refresh the display when filter parameters change
 *
 * The function only updates filters when the frequency or filter type changes
 * to optimize performance.
 *
 * Supported filter types:
 * - LOWSHELF: Low shelf filter with configurable frequency, gain, and Q
 * - HIGHSHELF: High shelf filter with configurable frequency, gain, and Q
 * - PEAKINGEQ: Peaking EQ filter with configurable frequency, Q, and gain
 * - BYPASS: Bypass filter (no effect)
 */
void loop(void)
{
  static uint32_t lastSampleRate = 0;
  static bool lastSampleRateStable = false;
  static time_t nextStatusPrint = 0;
  static time_t lastDACCheck = 0;
  static time_t lastMeterUpdate = 0;
  time_t currentTime = millis();

  if (currentTime - lastMeterUpdate >= 50)
  {
    if (controlValues.getUIMode() == UI_MODE_SIMPLE)
    {
      display.setVUMeterValue(peakMeter.getPeakLeft(), peakMeter.getPeakRight());
      //display.setVUMeterValue(rmsMeter.getRMSLeft(), rmsMeter.getRMSRight());
      peakMeter.reset();
      display.commit();
    } else if (controlValues.getUIMode() == UI_MODE_FFT)
    {
      float tdLeft[FFT_DISPLAY_BINS];
      float tdRight[FFT_DISPLAY_BINS];
      sample_t tdAvg[FFT_DISPLAY_BINS];
      spectrum.getNormalizedBins(tdLeft, tdRight, FFT_DISPLAY_BINS);
      for (int i = 0; i < FFT_DISPLAY_BINS; i++)
        tdAvg[i] = 0.5f * (tdLeft[i] + tdRight[i]);
      display.setFFTValues(tdAvg, true);
      display.commit();
    }
    lastMeterUpdate = currentTime;
  }

  // Is the DAC still connected? If not, blink the LED and keep checking until it comes back
  checkDACConnection(currentTime, lastDACCheck);
  if (!dacRunning)
  {
    blinkLED();
  }

  updateSampleRateStatus(lastSampleRate, lastSampleRateStable);
  printStatusIfNeeded(currentTime, nextStatusPrint);
  updateClipIndicator();
  tickControls();

  // If the selected band changed, we need to sync the encoder positions to the new band's settings

  readControlValues(controlValues);
  // Serial.printf("DIrty flags - dirty: %d, bandChanged: %d, filterChanged: %d\n",
  //               controlValues.isDirty(),
  //               controlValues.isBandChanged(),
  //               controlValues.isFilterChanged());
  if (!controlValues.isDirty())
  {
    saveSettingsIfNeeded(currentTime);
    return;
  }
  if (controlValues.isFilterChanged())
  {
    syncEncoders();
    updateFilter(controlValues.getSelectedBand());
  }
  gain.setGain(powf(10.0f, controlValues.getMasterGain() / 20.0f));
  dac.setVolumeDB(controlValues.getVolume());
  display.update(controlValues);
  display.commit();
  controlValues.reset();

  saveNeeded = true;
}
