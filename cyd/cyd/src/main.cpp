/*  Rui Santos & Sara Santos - Random Nerd Tutorials
    THIS EXAMPLE WAS TESTED WITH THE FOLLOWING HARDWARE:
    1) ESP32-2432S028R 2.8 inch 240×320 also known as the Cheap Yellow Display (CYD): https://makeradvisor.com/tools/cyd-cheap-yellow-display-esp32-2432s028r/
      SET UP INSTRUCTIONS: https://RandomNerdTutorials.com/cyd/
    2) REGULAR ESP32 Dev Board + 2.8 inch 240x320 TFT Display: https://makeradvisor.com/tools/2-8-inch-ili9341-tft-240x320/ and https://makeradvisor.com/tools/esp32-dev-board-wi-fi-bluetooth/
      SET UP INSTRUCTIONS: https://RandomNerdTutorials.com/esp32-tft/
    Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
    The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

#include <SPI.h>

/*  Install the "TFT_eSPI" library by Bodmer to interface with the TFT Display - https://github.com/Bodmer/TFT_eSPI
    *** IMPORTANT: User_Setup.h available on the internet will probably NOT work with the examples available at Random Nerd Tutorials ***
    *** YOU MUST USE THE User_Setup.h FILE PROVIDED IN THE LINK BELOW IN ORDER TO USE THE EXAMPLES FROM RANDOM NERD TUTORIALS ***
    FULL INSTRUCTIONS AVAILABLE ON HOW CONFIGURE THE LIBRARY: https://RandomNerdTutorials.com/cyd/ or https://RandomNerdTutorials.com/esp32-tft/   */
#include <TFT_eSPI.h>
#include <TFT_eWidget.h>
#include <esp_mac.h>

// Install the "XPT2046_Touchscreen" library by Paul Stoffregen to use the Touchscreen - https://github.com/PaulStoffregen/XPT2046_Touchscreen
// Note: this library doesn't require further configuration
#include <XPT2046_Touchscreen.h>

#include <complex.h>
#include <Wire.h>

#include "packets.h"

// Touchscreen pins
#define XPT2046_IRQ 36  // T_IRQ
#define XPT2046_MOSI 32 // T_DIN
#define XPT2046_MISO 39 // T_OUT
#define XPT2046_CLK 25  // T_CLK
#define XPT2046_CS 33   // T_CS

TFT_eSPI tft = TFT_eSPI(); // Create tft object

GraphWidget gr = GraphWidget(&tft);
TraceWidget tr = TraceWidget(&gr);

#define MAX_GRAPH_FREQUENCY 20000.0 // Maximum frequency for graph x axis
#define MIN_GRAPH_FREQUENCY 0.0     // Minimum frequency for graph x axis
#define MIN_GRAPH_AMPLITUDE -15.0   // Minimum amplitude for graph y axis
#define MAX_GRAPH_AMPLITUDE 15.0    // Maximum amplitude for graph y axis

void initGraph()
{
  // Graph area is 200 pixels wide, 150 high, dark grey background
  gr.createGraph(200, 150, tft.color565(5, 5, 5));

  // x scale units is from 0 to 100, y scale units is -50 to 50
  gr.setGraphScale(0, MAX_GRAPH_FREQUENCY + 1, MIN_GRAPH_AMPLITUDE, MAX_GRAPH_AMPLITUDE);

  // X grid starts at 0 with lines every 10 x-scale units
  // Y grid starts at -50 with lines every 25 y-scale units
  // blue grid
  gr.setGraphGrid(0, 4000, MIN_GRAPH_AMPLITUDE, 5.0, TFT_BLUE);

  // Draw the x axis scale
  tft.setTextDatum(TC_DATUM); // Top centre text datum
  tft.drawNumber(0, gr.getPointX(0) + 43, gr.getPointY(-15.0) + 13);
  tft.drawNumber(4000, gr.getPointX(4000) + 43, gr.getPointY(-15.0) + 13);
  tft.drawNumber(8000, gr.getPointX(8000) + 43, gr.getPointY(-15.0) + 13);
  tft.drawNumber(12000, gr.getPointX(12000) + 43, gr.getPointY(-15.0) + 13);
  tft.drawNumber(16000, gr.getPointX(16000) + 43, gr.getPointY(-15.0) + 13);
  tft.drawNumber(20000, gr.getPointX(20000) + 43, gr.getPointY(-15.0) + 13);

  // Draw the y axis scale
  tft.setTextDatum(MR_DATUM); // Middle right text datum
  tft.drawNumber(-15, gr.getPointX(0.0) + 32, gr.getPointY(-15.0) + 13);
  tft.drawNumber(0, gr.getPointX(0.0) + 32, gr.getPointY(0.0) + 13);
  tft.drawNumber(15, gr.getPointX(0.0) + 32, gr.getPointY(15.0) + 13);

  // Draw empty graph, top left corner at 40,10 on TFT
  gr.drawGraph(40, 10);
}

void updateGraph(float *x, float *y, int length)
{
  // Clear the previous graph by redrawing the background
  gr.drawGraph(40, 10);

  // Add the new line to the graph in yellow
  for (int i = 0; i < length - 1; i++)
  {
    tr.addPoint(x[i], y[i]);
  }
  tr.startTrace(TFT_YELLOW);
}

float getBiquadGain(float f, float fs, float a1, float a2, float b0, float b1, float b2)
{
  float w = 2.0 * M_PI * f / fs;
  std::complex<float> jw(0, w);
  std::complex<float> numerator = b0 + b1 * exp(-jw) + b2 * exp(-2.0f * jw);
  std::complex<float> denominator = 1.0f + a1 * exp(-jw) + a2 * exp(-2.0f * jw);
  return log10(std::abs(numerator / denominator)) * 20.0f;
}

void updateFilterCoeffs(float *values) {
  const int dataLength = 100;
  float freqResponse[dataLength];
  float freq[dataLength];
  const float maxFreq = 20000.0f;
  //Serial.printf("%f, %f, %f, %f, %f\n", values[4], values[5], values[1], values[2], values[3]);
  for (int i = 1; i < dataLength; i++)
  {
    freq[i] = float(i) / float(dataLength) * maxFreq;
    freqResponse[i] = getBiquadGain(freq[i], 44100.0f, values[4], values[5], values[1], values[2], values[3]);
  }
  updateGraph(freq, freqResponse, dataLength);
}

void updateFilterParameters(float *values) {
  // Extract set gain, Q and frequency
  float index = values[0];
  int filterType = int(values[1]);
  float gain = values[2];
  float Q = values[3];
  float frequency = values[4];
  tft.setTextColor(TFT_WHITE, TFT_BLACK); // White text with black background 
  tft.setTextSize(1);
  tft.setCursor(10, 200);
  tft.printf("Gain: %.2f dB  ", gain);
  tft.setCursor(120, 200);
  tft.printf("Q: %.2f", Q);
  tft.setCursor(190, 200);
  tft.printf("Freq: %.2f Hz  ", frequency);
}

void onReceive(int byteCount)
{
  char receivedData[256] = "";
  char *ptr = receivedData;
  while (Wire.available())
  {
    uint8_t c = Wire.read();
    *ptr++ = (char)c;
  }
  *ptr = '\0'; // Null-terminate the string
  //Serial.printf("Received data from DSP: %s\n", receivedData);
  
  // Parse comma-separaed string into 5 float values
  float values[8];
  int index = 0;
  char *token = strtok(receivedData, ",");
  while (token != NULL && index < 8)
  {
    values[index++] = atof(token);
    token = strtok(NULL, ",");
  } 
  int packetType = (int)values[0];
  switch(packetType)
  {
    case PACKET_COEFFS: // Filter parameters  update
      updateFilterCoeffs(values+1);
      break;
    case PACKET_PARAMS: // Other packet types can be handled here
      updateFilterParameters(values+1);
      break;
    default:
      Serial.println("Unknown packet type received");
      return;
  }
}

void setup()
{
  Serial.begin(115200);
  tft.init();
  tft.setRotation(3);

#ifdef ST7796_DRIVER
  // Deal with cheap ST7796 displays that have incorrect MADCTL settings
  tft.setRotation(3);
  tft.begin_nin_write();
  tft.writecommand(ST7796_MADCTL);
  tft.writedata(ST7796_MADCTL_MV | ST7796_MADCTL_MX | ST7796_MADCTL_BGR);
  tft.end_nin_write();
  tft.invertDisplay(true);
#endif

  tft.fillScreen(TFT_BLACK); // Clear screen to black
  // tft.setCursor(20, 20);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  initGraph();

  // Initialize communication with the DSP
  Wire.onReceive(onReceive);
  Wire.setPins(32, 25);
  Wire.setBufferSize(1024);
  Wire.begin(0xb1ce);
}

void loop()
{
  // Nothing to do here, all the work is done in the onReceive() function 
}
