/**
 * @file main.cpp
 * @brief Teensy-based parametric equalizer with rotary encoder controls and I2S audio output.
 *
 * This file implements a real-time audio parametric equalizer system using the Teensy Audio Library.
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
#include <Wire.h>
#include <SD.h>
#include <SerialFlash.h>
#include <OneButton.h>
#include <ErriezCRC32.h>
#include <stdlib.h>
#include "../../common/filter.h"
#include "../../common/packets.h"
#include "audio_pipeline/filter_biquad_f.h"
#include "audio_pipeline/audio_square_wave.h"
#include "audio_pipeline/audio_gain.h"
#include "AcceleratedEncoder.h"
#include "netconv.h"
#include "persistence.h"

// Audio interrupt control macros
#define AudioNoInterrupts() (NVIC_DISABLE_IRQ(IRQ_SOFTWARE))
#define AudioInterrupts()   (NVIC_ENABLE_IRQ(IRQ_SOFTWARE))


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
#define MASTER_GAIN_PIN_A 1
#define MASTER_GAIN_PIN_B 2

// Settings save interval
#define SAVE_INTERVAL_MS 5000

// Dely before clearing clip indication
#define CLIP_CLEAR_DELAY_MS 100

// Rotary encoders
AcceleratedEncoder fcSelector(FC_PIN_A, FC_PIN_B);
AcceleratedEncoder gainSelector(GAIN_PIN_A, GAIN_PIN_B);
AcceleratedEncoder qSelector(Q_PIN_A, Q_PIN_B);
AcceleratedEncoder masterGainSelector(MASTER_GAIN_PIN_A, MASTER_GAIN_PIN_B);

// Pushbuttons
OneButton filterSelectButton(FILTER_INDEX_PIN, true);
OneButton filterTypeSelectButton(FILTER_TYPE_PIN, true);
OneButton displayModeButton(FILTER_MODE_PIN, true);

AudioSquareWave waveform;

// The filters
AudioFilterBiquadFloat filter;

AudioGain gain;

// User settings
FilterSettings filterSettings[FILTER_BANDS];
int selectedFilterBand = LOW_BAND;
int displayMode = DISPLAY_MODE_INDIVIDUAL;
float masterGain = 0.0f;

// Last time settings were saved to EEPROM
unsigned long lastSaveTime = 0;
bool saveNeeded = false;

// Clipping state
typedef enum ClipState
{
  NO_CLIP,
  CLIP_TRIGGERED,
  CLIP_ACTIVE
} ClipState;

ClipState clipState = NO_CLIP;
uint32_t clipTimestamp = 0;

/**
 * @brief Sends a packet to the display device via I2C communication.
 *
 * This function calculates a CRC32 checksum for the packet (excluding the checksum field),
 * converts it to network byte order, and transmits the packet to the display device
 * at I2C address 0xb1ce.
 *
 * @param packet Pointer to the Packet structure to be sent. The checksum field of the
 *               packet will be updated before transmission.
 *
 * @note The checksum is calculated over the entire packet except the last 4 bytes
 *       (the checksum field itself).
 * @note This function uses the Wire library for I2C communication.
 */
void sendToDisplay(Packet *packet)
{
  packet->checksum = htonl(crc32Buffer((uint8_t *)packet, sizeof(Packet) - 4));
  AudioNoInterrupts();
  Wire.beginTransmission(0xb1ce);
  Wire.send((uint8_t *)packet, sizeof(Packet));
  Wire.endTransmission();
  AudioInterrupts();
}

/**
 * @brief Updates the display with current filter coefficients and parameters.
 *
 * This function sends two packets to the display:
 * 1. A PACKET_COEFFS packet containing the biquad filter coefficients (b0, b1, b2, a1, a2)
 *    for the currently selected filter band, converted to network byte order.
 * 2. A PACKET_PARAMS packet containing the filter parameters (type, frequency, Q, gain)
 *    for the currently selected filter band.
 *
 * @note When VERBOSE is defined, the filter coefficients are printed to the serial port
 *       for debugging purposes.
 * @note All floating-point values are converted to network byte order using htonf()
 *       before transmission.
 *
 * @see sendToDisplay()
 * @see filterLeft.getCoefficients()
 */
void updateDisplay()
{
  // Create packet with coefficients
  Packet packet = {};
  packet.packetType = PACKET_COEFFS;
  packet.selectedFilterBand = selectedFilterBand;
  packet.displayMode = displayMode;
  for (int i = 0; i < FILTER_BANDS; i++)
  {
    const double *coeffs = filter.getCoefficients(i);
    packet.data.filters.coeffs[i].b0 = htonf(coeffs[0]);
    packet.data.filters.coeffs[i].b1 = htonf(coeffs[1]);
    packet.data.filters.coeffs[i].b2 = htonf(coeffs[2]);
    packet.data.filters.coeffs[i].a1 = htonf(-coeffs[3]); // Negated due to difference equation form
    packet.data.filters.coeffs[i].a2 = htonf(-coeffs[4]);
  }
  packet.data.filters.masterGain = htonf(masterGain);
  sendToDisplay(&packet);

  // Send filter parameters to display
  FilterSettings *settings = &filterSettings[selectedFilterBand];
  packet = {};
  packet.packetType = PACKET_PARAMS;
  packet.selectedFilterBand = selectedFilterBand;
  packet.displayMode = displayMode;
  for (int i = 0; i < FILTER_BANDS; i++)
  {
    packet.selectedFilterBand = selectedFilterBand;
    packet.data.params[i].filterType = filterSettings[i].type;
    packet.data.params[i].frequency = htonf(filterSettings[i].frequency);
    packet.data.params[i].Q = htonf(filterSettings[i].Q);
    packet.data.params[i].gain = htonf(filterSettings[i].gain);
  }
  sendToDisplay(&packet);
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
  FilterSettings *settings = &filterSettings[selected];
  switch (settings->type)
  {
  case LOWSHELF:
    filter.setLowShelf(selected, settings->frequency, settings->gain, settings->Q);
    break;
  case HIGHSHELF:
    filter.setHighShelf(selected, settings->frequency, settings->gain, settings->Q);
    break;
  case PEAKINGEQ:
    filter.setPeakingEQ(selected, settings->frequency, settings->Q, settings->gain);
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

  // Build audio pipeline
  // Source -> Gain -> Filter -> I2S
  gain.addReceiver(&filter);
  filter.addReceiver(AudioController::getInstance());

  // AudioController::getInstance()->addReceiver(&waveform);
  AudioController::getInstance()->addReceiver(&gain);
  AudioController::getInstance()->setClipDetector(clipDetected);
  gain.setGain(0.5f); // Reduce volume to avoid clipping

  InitI2s(true);
  waveform.setAmplitude(0.5f);
  waveform.setFrequency(980.0f); // 980 Hz test tone
  // waveform.addReceiver(&gain);

  // need to wait a bit before configuring codec, otherwise something weird happens and there's no output...
  delay(100);

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
  pinMode(MASTER_GAIN_PIN_A, INPUT_PULLUP);
  pinMode(MASTER_GAIN_PIN_B, INPUT_PULLUP);

  // Initialize clip LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Set up pushbuttons
  filterTypeSelectButton.attachPress([]()
                                     { filterSettings[selectedFilterBand].type = (filterSettings[selectedFilterBand].type + 1) % NUM_FILTER_TYPES;
    Serial.printf("Selected filter type: %d\n", filterSettings[selectedFilterBand].type); });

  filterSelectButton.attachPress([]()
                                 {  selectedFilterBand = (selectedFilterBand + 1) % FILTER_BANDS;
  Serial.printf("Selected filter band: %d\n", selectedFilterBand); });

  displayModeButton.attachPress([]()
                                { displayMode = (displayMode + 1) % 2;
    Serial.printf("Selected display mode: %d\n", displayMode); });

  // Initialize S2C
  Wire.begin();

  // Set up Q control on A10
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  pinMode(A12, INPUT);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH); // Chip select high

  // Reset last save time
  lastSaveTime = millis();

  // Load persisted settings
  PersistedSettings persistedSettings;
  if (loadSettings(persistedSettings))
  {
    for (int i = 0; i < FILTER_BANDS; i++)
    {
      filterSettings[i] = persistedSettings.filterSettings[i];
    }
    selectedFilterBand = persistedSettings.selectedFilterBand;
    displayMode = persistedSettings.displayMode;
    Serial.println("Loaded persisted settings from EEPROM.");
  }
  else
  {
    // Initialize default settings
    Serial.println("No persisted settings found, using defaults.");
    for (int i = 0; i < FILTER_BANDS; i++)
    {
      filterSettings[i].frequency = 1234.0f; // 1 kHz
      filterSettings[i].Q = 0.707f;          // Q=0.707
      filterSettings[i].gain = 0.0f;         // 0 dB
      filterSettings[i].type = BYPASS;       // Bypass
    }
  }

  // Set initial positions
  FilterSettings *settings = &filterSettings[selectedFilterBand];
  selectedFilterBand = 0;
  fcSelector.setAcceleratedPosition(log10(settings->frequency) / LOG_FREQ_STEP);
  fcSelector.setEndpoints(log10(MIN_FREQUENCY) / LOG_FREQ_STEP, log10(MAX_FREQUENCY) / LOG_FREQ_STEP);
  gainSelector.setAcceleratedPosition(settings->gain / GAIN_STEP_DB);
  gainSelector.setEndpoints(MIN_GAIN_DB / GAIN_STEP_DB, MAX_GAIN_DB / GAIN_STEP_DB);
  qSelector.setAcceleratedPosition(settings->Q / Q_STEP);
  qSelector.setEndpoints(MIN_Q / Q_STEP, MAX_Q / Q_STEP);
  masterGainSelector.setAcceleratedPosition(0);
  masterGainSelector.setEndpoints(MASTER_GAIN_MIN / MASTER_GAIN_STEP_DB, MASTER_GAIN_MAX / MASTER_GAIN_STEP_DB); // -20 dB to 20 dB
  masterGainSelector.setAcceleratedPosition(0); // 0 dB

  // Initial display update
  updateAllFilters();
  updateDisplay();
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
 * The function only updates filters when the frequency or filter type changes to optimize performance.
 *
 * Supported filter types:
 * - LOWSHELF: Low shelf filter with configurable frequency, gain, and Q
 * - HIGHSHELF: High shelf filter with configurable frequency, gain, and Q
 * - PEAKINGEQ: Peaking EQ filter with configurable frequency, Q, and gain
 * - BYPASS: Bypass filter (no effect)
 */
void loop(void)
{
  static time_t nextStatusPrint = 0;
  time_t currentTime = millis();

  // Print CPU load every 2 seconds
  if (currentTime >= nextStatusPrint)
  {
    Serial.printf("CPU Load: %.2f%%, Sample rate: %u Hz, stable: %s\n", Timers::GetCpuLoad() * 100.0f, AudioController::getStandardizedSampleRate(), AudioController::isSampleRateStable() ? "true" : "false");
    nextStatusPrint = currentTime + 2000; // Print every 2 seconds
  }

  // Update clip indicator
  if (clipState == CLIP_TRIGGERED)
  {
    clipState = CLIP_ACTIVE;
    clipTimestamp = millis();
    digitalWrite(LED_BUILTIN, HIGH);
    Packet p;
    p.packetType = PACKET_CLIP_ALERT;
    p.data.clipAlert.clipped = 1;
    //sendToDisplay(&p);
  }
  else if (clipState == CLIP_ACTIVE)
  {
    if ((millis() - clipTimestamp) > CLIP_CLEAR_DELAY_MS)
    {
      clipState = NO_CLIP;
      Packet p;
      p.packetType = PACKET_CLIP_ALERT;
      p.data.clipAlert.clipped = 0;
      //sendToDisplay(&p);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  // Tick rotary encoders
  FilterSettings *settings = &filterSettings[selectedFilterBand];
  int oldFilterType = settings->type;
  int oldSelectedBand = selectedFilterBand;
  int oldDisplayMode = displayMode;
  float oldMasterGain = masterGain;
  fcSelector.tick();
  gainSelector.tick();
  qSelector.tick();
  filterTypeSelectButton.tick(); // May change selectedFilterType!
  filterSelectButton.tick();     // May change selectedFilterBand!
  displayModeButton.tick();      // May change displayMode!
  masterGainSelector.tick();     // May change master gain!

  // If the filter band changed, we have to update all the rotary encoders to reflect current settings
  if (oldSelectedBand != selectedFilterBand)
  {
    settings = &filterSettings[selectedFilterBand];
    // Update rotary encoders to reflect current settings
    fcSelector.setAcceleratedPosition(map(log10(settings->frequency), LOG_MIN_FREQUENCY, LOG_MAX_FREQUENCY,
                                          LOG_MIN_FREQUENCY / LOG_FREQ_STEP, LOG_MAX_FREQUENCY / LOG_FREQ_STEP));
    gainSelector.setAcceleratedPosition(
        map(settings->gain, MIN_GAIN_DB, MAX_GAIN_DB, MIN_GAIN_DB / GAIN_STEP_DB, MAX_GAIN_DB / GAIN_STEP_DB));
    qSelector.setAcceleratedPosition(
        map(settings->Q, MIN_Q, MAX_Q, MIN_Q / Q_STEP, MAX_Q / Q_STEP));
  }

  float newFrequency = exp10(map(float(fcSelector.geAcceleratedPosition()), LOG_MIN_FREQUENCY / LOG_FREQ_STEP,
                                 LOG_MAX_FREQUENCY / LOG_FREQ_STEP, LOG_MIN_FREQUENCY, LOG_MAX_FREQUENCY));
  float newQ = map(float(qSelector.geAcceleratedPosition()), MIN_Q / Q_STEP, MAX_Q / Q_STEP, MIN_Q, MAX_Q);
  float newGain = map(float(gainSelector.geAcceleratedPosition()), MIN_GAIN_DB / GAIN_STEP_DB,
                      MAX_GAIN_DB / GAIN_STEP_DB, MIN_GAIN_DB, MAX_GAIN_DB);
  float newMasterGain = map(float(masterGainSelector.geAcceleratedPosition()), MASTER_GAIN_MIN / MASTER_GAIN_STEP_DB,
                             MASTER_GAIN_MAX / MASTER_GAIN_STEP_DB, MASTER_GAIN_MIN, MASTER_GAIN_MAX);

  // Update filters only if parameters changed
  settings = &filterSettings[selectedFilterBand]; // The band could have changed
  if (newFrequency == settings->frequency && oldFilterType == settings->type && newGain == settings->gain && newQ == settings->Q && 
    oldSelectedBand == selectedFilterBand && oldDisplayMode == displayMode && newMasterGain == oldMasterGain)
  {
    // Save to EEPROM if needed
    unsigned long currentTime = millis();
    if (saveNeeded && currentTime - lastSaveTime > SAVE_INTERVAL_MS)
    {
      PersistedSettings persistedSettings; 
      for (int i = 0; i < FILTER_BANDS; i++)
      {
        persistedSettings.filterSettings[i] = filterSettings[i];
      }
      persistedSettings.selectedFilterBand = selectedFilterBand;
      persistedSettings.displayMode = displayMode;
      saveSettings(persistedSettings);
      lastSaveTime = currentTime;
      Serial.println("Settings saved to EEPROM.");
      saveNeeded = false;
    }
    return;
  }
  Serial.printf("Frequency: %f Hz, Master gain: %f dB\n", newFrequency, newMasterGain);
  settings->frequency = newFrequency;
  settings->gain = newGain;
  settings->Q = newQ;
  masterGain = newMasterGain;
  gain.setGain(powf(10.0f, masterGain / 20.0f)); // Convert dB to linear

  // Set up filter according to type
  updateFilter(selectedFilterBand);
  updateDisplay();
  saveNeeded = true;
} 