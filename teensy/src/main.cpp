#include <Audio.h>
#include <Wire.h>
#include <Wire.h>
#include <SD.h>
#include <SerialFlash.h>
#include <stdlib.h>
#include "filter_biquad_f.h"
#include "packets.h"

// Band selection
#define LOW_BAND 0
#define MID_BAND 1
#define HIGH_BAND 2

// Filter type selection
#define LOWSHELF 0
#define HIGHSHELF 1
#define PEAKINGEQ 2

AudioSynthWaveform        waveform;       

// The filters
AudioFilterBiquadFloat    filterLeft;        
AudioFilterBiquadFloat    filterRight;        

AudioConnection           patchCordL1(waveform, 0, filterLeft, 0);
AudioConnection           patchCordR1(waveform, 0, filterRight, 0);

AudioMixer4               mixer2;            
AudioMixer4               mixer1;            
AudioOutputI2S            i2s;            

// Conect filter chains to mixers and I2S output
AudioConnection           patchCord1(filterLeft, 0, mixer1, 0);
AudioConnection           patchCord2(filterRight, 0, mixer2, 0);
AudioConnection           patchCord3(mixer2,    0, i2s,    1);
AudioConnection           patchCord4(mixer1,    0, i2s,    0);


void setup(void)
{
    Serial.begin(115000);
    AudioMemory(12);
    waveform.begin(WAVEFORM_SQUARE);
    waveform.amplitude(0.3f);
    waveform.frequency(980.0f);

    // Initialize SPI
    Wire.begin();

    // Set up Q control on A10
    pinMode(A10, INPUT);   
    pinMode(A11, INPUT);
    pinMode(A12, INPUT);
    pinMode(10, OUTPUT);
    digitalWrite(10, HIGH); // Chip select high
 
    mixer1.gain(0, 0.5);    // Left channel
    mixer2.gain(0, 0.5);    // Right channel
}

int selectedFilterBand = LOW_BAND;
int selectedFilterType = PEAKINGEQ;

void sendToDisplay(int packet_type, String s) {
    Wire.beginTransmission(0xb1ce);
    Wire.write(String(packet_type).c_str());
    Wire.write(',');
    Wire.write(s.c_str());
    Wire.endTransmission();
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
    sensorValue = analogRead(A12);
    float frequency = map(float(sensorValue), 0.0, 1023.0, 20.0, 20000.0);
    Serial.printf("Frequency: %f Hz\n", frequency); 

    // Select the filter to work with
    

    // Set up filter according to type
    switch(selectedFilterType) {
        case LOWSHELF:
            filterLeft.setLowShelf(selectedFilterBand, frequency, gain, q);
            filterRight.setLowShelf(selectedFilterBand, frequency, gain, q);
            break;
        case HIGHSHELF: 
            filterLeft.setHighShelf(selectedFilterBand, frequency, gain, q);
            filterRight.setHighShelf(selectedFilterBand, frequency, gain, q);
            break;
        case PEAKINGEQ:
            filterLeft.setPeakingEQ(selectedFilterBand, frequency, q, gain);
            filterRight.setPeakingEQ(selectedFilterBand, frequency, q, gain);
            break;
    }
 
    // Get filter coefficients and transfer via SPI as a comma separated list
    const float* coefs = filterLeft.getCoefficients(0);
    String buf = String(selectedFilterBand) + ",";
    for (int i = 0; i < 5; i++) {
        String coefStr = String(coefs[i], 6);
        buf += coefStr;
        buf += ",";
    }
    sendToDisplay(PACKET_COEFFS, buf);
    delay(100);

    // Send filter parameters to display
    buf = String(selectedFilterBand) + ",";
    buf += String(selectedFilterType) + ",";
    buf += String(gain, 4);
    buf += ",";
    buf += String(q, 4);
    buf += ",";
    buf += String(frequency, 4);
    sendToDisplay(PACKET_PARAMS, buf);
    delay(90);
}