/*
  Demo of the audio sweep function.
  The user specifies the amplitude,
  start and end frequencies (which can sweep up or down)
  and the length of time of the sweep.

  Modified to eliminate the audio shield, and use Max98357A mono I2S chip.
  https://smile.amazon.com/gp/product/B07PS653CD/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1

  Pins:        Teensy 4.0    Teensy 3.x

  LRCLK:    Pin 20/A6    Pin 23/A9
  BCLK:        Pin 21/A7    Pin 9
  DIN:        Pin 7        Pin 22/A8
  Gain:        see below    see below
  Shutdown:    N/C        N/C
  Ground:    Ground        Ground
  VIN:        5v        5v

  Other I2S pins not used by the Max98357A device:

  MCLK        Pin 23        Pin 11
  DOUT        Pin 8        Pin 13

  Gain setting:

  15dB    if a 100K resistor is connected between GAIN and GND
  12dB    if GAIN is connected directly to GND
   9dB    if GAIN is not connected to anything (this is the default)
   6dB    if GAIN is conneted directly to Vin
   3dB    if a 100K resistor is connected between GAIN and Vin.

  SD setting (documentation from the Adafruit board)

  This pin is used for shutdown mode but is also used for setting which channel
  is output. It's a little confusing but essentially:

  * If SD is connected to ground directly (voltage is under 0.16V) then the amp
    is shut down

  * If the voltage on SD is between 0.16V and 0.77V then the output is (Left +
    Right)/2, that is the stereo average.

  * If the voltage on SD is between 0.77V and 1.4V then the output is just the
    Right channel

  * If the voltage on SD is higher than 1.4V then the output is the Left
    channel.

    This is compounded by an internal 100K pulldown resistor on SD so you need
    to use a pullup resistor on SD to balance out the 100K internal pulldown.

  Or alternatively, use the HiLetgo PCM5102 I2S IIS Lossless Digital Audio DAC
  Decoder which provides stereo output:
  https://smile.amazon.com/gp/product/B07Q9K5MT8/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1  */

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include "filter_biquad_f.h"

// GUItool: begin automatically generated code (edited by meissner afterwards).
AudioSynthWaveform        waveform;        //xy=99,198
AudioFilterBiquadFloat    biquad1;        //xy=199,198
AudioMixer4               mixer2;            //xy=280,253
AudioMixer4               mixer1;            //xy=280,175
AudioOutputI2S            i2s;            //xy=452,189

AudioConnection           patchCord0(waveform, 0, biquad1, 0);
AudioConnection           patchCord1(biquad1, 0, mixer1, 0);
AudioConnection           patchCord2(biquad1, 0, mixer2, 0);
AudioConnection           patchCord3(mixer2,    0, i2s,    1);
AudioConnection           patchCord4(mixer1,    0, i2s,    0);
// GUItool: end automatically generated code

const float    t_ampx    = 0.8;
const int     t_lox    = 1000;
const int    t_hix    = 22000;
const float    t_timex    = 100000;        // Length of time for the sweep in seconds


void setup(void)
{
    Serial.begin(9600);
    AudioMemory(12);
    waveform.begin(WAVEFORM_SQUARE);
    waveform.amplitude(0.3f);
    waveform.frequency(980.0f);

    // Set up Q control on A10
    pinMode(A10, INPUT);   
    pinMode(A11, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    //biquad1.setPeakingEQ(0, 5000.0f, 60.0f, 50.0f); 
    //biquad1.setLowpass(0, 1000.0f, 0.7071f);
    

    mixer1.gain(0, 0.5);    // Left channel
    mixer2.gain(0, 0.5);    // Right channel
}

void loop (void)
{
    // Read the Q control
    int sensorValue = analogRead(A10);
    float gain = map(float(sensorValue), 0.0, 1023.0, -55.0, 15.0);
    Serial.printf("Sensor value: %d, Gain: %f\n", sensorValue, gain);
    sensorValue = analogRead(A11);
    float q = map(float(sensorValue), 0.0, 1023.0, 0.1, 20);
    Serial.printf("Gain: %f, Q: %f, s: %f\n", gain, q, sensorValue);
    biquad1.setPeakingEQ(0, 980.0f * 5.0f, q, gain); 
    float rawGain = pow(10.0, gain/20.0f);
    //waveform.amplitude(0.5f / rawGain);    // Left channel
    // mixer2.gain(0, 0.5f / rawGain);    // Right channel
    //biquad1.setLowpass(0, 1000.0f, qValue);
    delay(100);
}