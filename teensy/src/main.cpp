#include <Audio.h>
#include <Wire.h>
#include <Wire.h>
#include <SD.h>
#include <SerialFlash.h>
#include <OneButton.h>
#include <ErriezCRC32.h>
#include <stdlib.h>
#include "filter_biquad_f.h"
#include "packets.h"
#include "AcceleratedEncoder.h"
#include "netconv.h"

// Band selection
#define LOW_BAND 0
#define MID_BAND 1
#define HIGH_BAND 2

// Filter type selection
#define LOWSHELF 0
#define HIGHSHELF 1
#define PEAKINGEQ 2

// Rotary encoder pins
#define FC_PIN_A 33
#define FC_PIN_B 34
#define FILTER_TYPE_PIN 35
#define GAIN_PIN_A 36
#define GAIN_PIN_B 37
#define FILTER_INDEX_PIN 38

#define Q_PIN_A 39
#define Q_PIN_B 40

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

// State variables
int selectedFilterBand = LOW_BAND;
int selectedFilterType = PEAKINGEQ;

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
  fcSelector.setEndpoints(2, 2000);
  fcSelector.setAcceleratedPosition(100); // Start at 1 kHz
  gainSelector.setPosition(PEAKINGEQ);
  qSelector.setPosition(707); // Q=0.707

  // Set up pushbuttons
  filterTypeSelectButton.attachPress([]()
                                     {
    selectedFilterType = (selectedFilterType + 1) % 3;
    Serial.printf("Selected filter type: %d\n", selectedFilterType); });

  // Initialize SPI
  Wire.begin();

  // Set up Q control on A10
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  pinMode(A12, INPUT);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH); // Chip select high

  mixer1.gain(0, 0.5); // Left channel
  mixer2.gain(0, 0.5); // Right channel
}

void sendToDisplay(Packet *packet)
{
  packet->checksum = crc32Buffer((uint8_t *)packet, sizeof(Packet) - 4);
  Wire.beginTransmission(0xb1ce);
  Wire.send((uint8_t *)packet, sizeof(Packet));
  Wire.endTransmission();
}

float gain = -10.0f;
float q = 20.0f;
float frequency = 1000.0f;

void loop(void)
{
// Read the Q control
#ifdef ANALOG_CONTROLS
  int sensorValue = analogRead(A10);
  float gain = map(float(sensorValue), 0.0, 1023.0, -55.0, 15.0);
  Serial.printf("Sensor value: %d, Gain: %f\n", sensorValue, gain);
  sensorValue = analogRead(A11);
  float q = map(float(sensorValue), 0.0, 1 023.0, 0.1, 20);
  Serial.printf("Gain: %f, Q: %f, s: %f\n", gain, q, sensorValue);
  sensorValue = analogRead(A12);
  float frequency = map(float(sensorValue), 0.0, 1023.0, 20.0, 20000.0);
  Serial.printf("Frequency: %f Hz\n", frequency);
#endif

  // Update rotary encoders
  int oldFilterType = selectedFilterType;
  fcSelector.tick();
  gainSelector.tick();
  qSelector.tick();
  filterTypeSelectButton.tick(); // May change selectedFilterType!
  float newFrequency = map(float(fcSelector.geAcceleratedPosition()), 0, 2000, 0, 20000.0);
  if (newFrequency == frequency && selectedFilterType == oldFilterType)
  {
    return;
  }
  Serial.printf("Frequency: %f Hz\n", newFrequency);
  frequency = newFrequency;

  // Set up filter according to type
  switch (selectedFilterType)
  {
  case LOWSHELF:
    filterLeft.setLowShelf(selectedFilterBand, frequency, gain, q / 10.0f);
    filterRight.setLowShelf(selectedFilterBand, frequency, gain, q / 10.0f);
    break;
  case HIGHSHELF:
    filterLeft.setHighShelf(selectedFilterBand, frequency, gain, q / 10.0f);
    filterRight.setHighShelf(selectedFilterBand, frequency, gain, q / 10.0f);
    break;
  case PEAKINGEQ:
    filterLeft.setPeakingEQ(selectedFilterBand, frequency, q, gain);
    filterRight.setPeakingEQ(selectedFilterBand, frequency, q, gain);
    break;
  }

  // Create packet with coefficients
  const float *coefs = filterLeft.getCoefficients(0);
  Packet packet = {};
  packet.packetType = PACKET_COEFFS;
  packet.data.coeffs.filterIndex = selectedFilterBand;
  packet.data.coeffs.b0 = htonf(coefs[0]);
  packet.data.coeffs.b1 = htonf(coefs[1]);
  packet.data.coeffs.b2 = htonf(coefs[2]);  
  packet.data.coeffs.a1 = htonf(coefs[3]);
  packet.data.coeffs.a2 = htonf(coefs[4]);
  sendToDisplay(&packet );
  // delay(100);

  // Send filter parameters to display
  packet = {};
  packet.packetType = PACKET_PARAMS;
  packet.data.params.filterIndex = selectedFilterBand;
  packet.data.params.filterType = selectedFilterType;
  packet.data.params.frequency = htonf(frequency);
  packet.data.params.Q = htonf(q);
  packet.data.params.gain = htonf(gain);
  sendToDisplay(&packet);
}