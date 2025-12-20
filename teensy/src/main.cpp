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
#include <Audio.h>
#include <Wire.h>
#include <Wire.h>
#include <SD.h>
#include <SerialFlash.h>
#include <OneButton.h>
#include <ErriezCRC32.h>
#include <stdlib.h>
#include "filter.h"
#include "filter_biquad_f.h"
#include "packets.h"
#include "AcceleratedEncoder.h"
#include "netconv.h"
#include "persistence.h"

// Filter control endpoints and steps
#define MIN_GAIN_DB -15.0f
#define MAX_GAIN_DB 15.0f
#define GAIN_STEP_DB 0.1f
#define MIN_Q 0.1f
#define MAX_Q 5.0f
#define Q_STEP 0.1f
#define MIN_FREQUENCY 20.0f
#define MAX_FREQUENCY 20000.0f
#define FREQ_STEP 5.0f

// Rotary encoder pins
#define FC_PIN_A 33
#define FC_PIN_B 34
#define FILTER_TYPE_PIN 35
#define GAIN_PIN_A 36
#define GAIN_PIN_B 37
#define FILTER_INDEX_PIN 38

#define Q_PIN_A 39
#define Q_PIN_B 40

// Settings save interval
#define SAVE_INTERVAL_MS 5000

// Rotary encoders
AcceleratedEncoder fcSelector(FC_PIN_A, FC_PIN_B);
AcceleratedEncoder gainSelector(GAIN_PIN_A, GAIN_PIN_B);
AcceleratedEncoder qSelector(Q_PIN_A, Q_PIN_B);

// Pushbuttons
OneButton filterSelectButton(FILTER_INDEX_PIN, true);
OneButton filterTypeSelectButton(FILTER_TYPE_PIN, true);

AudioSynthWaveform waveform;

// The filters
AudioFilterBiquadFloat filterLeft;
AudioFilterBiquadFloat filterRight;

AudioConnection patchCordL1(waveform, 0, filterLeft, 0);
AudioConnection patchCordR1(waveform, 0, filterRight, 0);

AudioMixer4 mixer2;
AudioMixer4 mixer1;
AudioOutputI2S i2s;

// Conect filter chains to mixers and I2S output
AudioConnection patchCord1(filterLeft, 0, mixer1, 0);
AudioConnection patchCord2(filterRight, 0, mixer2, 0);
AudioConnection patchCord3(mixer2, 0, i2s, 1);
AudioConnection patchCord4(mixer1, 0, i2s, 0);

// User settings
volatile FilterSettings filterSettings[FILTER_BANDS];
int selectedFilterBand = LOW_BAND;

// Last time settings were saved to EEPROM
unsigned long lastSaveTime = 0;
bool saveNeeded = false;

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
  Wire.beginTransmission(0xb1ce);
  Wire.send((uint8_t *)packet, sizeof(Packet));
  Wire.endTransmission();
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
  const float *coeffs = filterLeft.getCoefficients(selectedFilterBand);
  Packet packet = {};
  packet.packetType = PACKET_COEFFS;
  packet.selectedFilterBand = selectedFilterBand;
  for(int i = 0; i< FILTER_BANDS; i++) {
    packet.data.coeffs[selectedFilterBand].b0 = htonf(coeffs[0]);
    packet.data.coeffs[selectedFilterBand].b1 = htonf(coeffs[1]);
    packet.data.coeffs[selectedFilterBand].b2 = htonf(coeffs[2]);
    packet.data.coeffs[selectedFilterBand].a1 = htonf(coeffs[3]);
    packet.data.coeffs[selectedFilterBand].a2 = htonf(coeffs[4]);
  }
  sendToDisplay(&packet);

  // Send filter parameters to display
  volatile FilterSettings *settings = &filterSettings[selectedFilterBand];
  packet = {};
  packet.packetType = PACKET_PARAMS;
  packet.selectedFilterBand = selectedFilterBand;
  for(int i = 0; i< FILTER_BANDS; i++) {
    packet.selectedFilterBand = selectedFilterBand;
    packet.data.params[selectedFilterBand].filterType = settings->type;
    packet.data.params[selectedFilterBand].frequency = htonf(settings->frequency);
    packet.data.params[selectedFilterBand].Q = htonf(settings->Q);
    packet.data.params[selectedFilterBand].gain = htonf(settings->gain);
  }
  sendToDisplay(&packet);
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
  AudioMemory(12);
  waveform.begin(WAVEFORM_SQUARE);
  waveform.amplitude(0.3f);
  waveform.frequency(980.0f);

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

  // Set initial positions
  fcSelector.setAcceleratedPosition(0);
  fcSelector.setEndpoints(MIN_FREQUENCY / FREQ_STEP, MAX_FREQUENCY / FREQ_STEP);
  fcSelector.setAcceleratedPosition(1000 / FREQ_STEP); // Start at 1 kHz
  gainSelector.setPosition(PEAKINGEQ);
  gainSelector.setEndpoints(MIN_GAIN_DB / GAIN_STEP_DB, MAX_GAIN_DB / GAIN_STEP_DB);
  gainSelector.setAcceleratedPosition(0); // Start at 0 dB
  qSelector.setPosition(707 / Q_STEP);    // Q=0.707
  qSelector.setEndpoints(MIN_Q / Q_STEP, MAX_Q / Q_STEP);

  // Set up pushbuttons
  filterTypeSelectButton.attachPress([]()
                                     {
    filterSettings[selectedFilterBand].type = (filterSettings[selectedFilterBand].type + 1) % 3;
    Serial.printf("Selected filter type: %d\n", filterSettings[selectedFilterBand].type); });

    filterSelectButton.attachPress([]()
                                     {  
    selectedFilterBand = (selectedFilterBand + 1) % FILTER_BANDS;
    Serial.printf("Selected filter band: %d\n", selectedFilterBand); });
                                  
  // Initialize S2C
  Wire.begin();

  // Set up Q control on A10
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  pinMode(A12, INPUT);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH); // Chip select high

  mixer1.gain(0, 0.5); // Left channel
  mixer2.gain(0, 0.5); // Right channel

  // Initial display update
  updateDisplay();

  // Reset last save time
  lastSaveTime = millis();
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
 */
void loop(void)
{
  // Tick rotary encoders
  volatile FilterSettings *settings = &filterSettings[selectedFilterBand];
  int oldFilterType = settings->type;
  int oldSelectedBand = selectedFilterBand;
  fcSelector.tick();
  gainSelector.tick();
  qSelector.tick();
  filterTypeSelectButton.tick(); // May change selectedFilterType!
  filterSelectButton.tick();     // May change selectedFilterBand!

  // If the filter band changed, we have to update all the rotary encoders to reflect current settings
  if (oldSelectedBand != selectedFilterBand)
  {
    settings = &filterSettings[selectedFilterBand];
    // Update rotary encoders to reflect current settings
    fcSelector.setAcceleratedPosition(map(settings->frequency, MIN_FREQUENCY, MAX_FREQUENCY,
                                          MIN_FREQUENCY / FREQ_STEP, MAX_FREQUENCY / FREQ_STEP));
    gainSelector.setAcceleratedPosition(
        map(settings->gain, MIN_GAIN_DB, MAX_GAIN_DB, MIN_GAIN_DB / GAIN_STEP_DB, MAX_GAIN_DB / GAIN_STEP_DB));
    qSelector.setAcceleratedPosition(
        map(settings->Q, MIN_Q, MAX_Q, MIN_Q / Q_STEP, MAX_Q / Q_STEP));
  }

  float newFrequency = map(float(fcSelector.geAcceleratedPosition()), MIN_FREQUENCY / FREQ_STEP,
                           MAX_FREQUENCY / FREQ_STEP, MIN_FREQUENCY, MAX_FREQUENCY);
  float newQ = map(float(qSelector.geAcceleratedPosition()), MIN_Q / Q_STEP, MAX_Q / Q_STEP, MIN_Q, MAX_Q);
  float newGain = map(float(gainSelector.geAcceleratedPosition()), MIN_GAIN_DB / GAIN_STEP_DB,
                      MAX_GAIN_DB / GAIN_STEP_DB, MIN_GAIN_DB, MAX_GAIN_DB);

  // Update filters only if parameters changed
  settings = &filterSettings[selectedFilterBand]; // The band could have changed
  if (newFrequency == settings->frequency && oldFilterType == settings->type && newGain == settings->gain 
    && newQ == settings->Q && oldSelectedBand == selectedFilterBand)
  {
    // Save to EEPROM if needed
    unsigned long currentTime = millis();
    if (saveNeeded && currentTime - lastSaveTime > SAVE_INTERVAL_MS)
    {
      PersistedSettings persistedSettings;
      persistedSettings.filterTypes[selectedFilterBand] = settings->type;
      persistedSettings.filterFrequencies[selectedFilterBand] = settings->frequency;
      persistedSettings.filterGains[selectedFilterBand] = settings->gain;
      persistedSettings.filterQs[selectedFilterBand] = settings->Q;
      saveSettings(persistedSettings);
      lastSaveTime = currentTime;
      Serial.println("Settings saved to EEPROM.");
      saveNeeded = false;
    }
    return;
  }
  Serial.printf("Frequency: %f Hz\n", newFrequency);
  settings->frequency = newFrequency;
  settings->gain = newGain;
  settings->Q = newQ;

  // Set up filter according to type
  switch (settings->type)
  {
  case LOWSHELF:
    filterLeft.setLowShelf(selectedFilterBand, settings->frequency, settings->gain, settings->Q);
    filterRight.setLowShelf(selectedFilterBand, settings->frequency, settings->gain, settings->Q);
    break;
  case HIGHSHELF:
    filterLeft.setHighShelf(selectedFilterBand, settings->frequency, settings->gain, settings->Q);
    filterRight.setHighShelf(selectedFilterBand, settings->frequency, settings->gain, settings->Q);
    break;
  case PEAKINGEQ:
    filterLeft.setPeakingEQ(selectedFilterBand, settings->frequency, settings->Q, settings->gain);
    filterRight.setPeakingEQ(selectedFilterBand, settings->frequency, settings->Q, settings->gain);
    break;
  }
  updateDisplay();
  saveNeeded = true;
}