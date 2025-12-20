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

#include "comms.h"
#include "netconv.h"

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

#define COL_BACKGROUND 0
#define COL_TEXT_BACKGROUND 1
#define COL_FOREGROUND 2
#define COL_GRID 3
#define COL_TRACE 4
#define COL_GRAPH_BACKGROUND 5
#define COL_ENABLED 6
#define COL_DISABLED 7
#define NUM_COLORS 8

#define NUM_FILTERS 3

#define SAMPLE_FREQ 44100.0f

const char *filterTypes[] = {
    "LoSh",
    "HiSh",
    "Peak"};

uint16_t darkMode[NUM_COLORS] = {
    TFT_BLACK,             // BACKGROUND
    TFT_BLACK,             // TEXT_BACGROUND
    TFT_WHITE,             // FOREGROUND
    TFT_DARKGREEN,         // GRID
    TFT_YELLOW,            // TRACE
    tft.color565(5, 5, 5), //  GRAPH_BACKGROUND
    TFT_GREEN,             // ENABLED
    TFT_GREY               // DISABLED
};

uint16_t lightMode[NUM_COLORS] = {
    TFT_WHITE,             // BACKGROUND
    TFT_WHITE,             // TEXT_BACKGROUND
    TFT_BLACK,             // FOREGROUND
    TFT_DARKGREEN,         // GRID
    TFT_YELLOW,            // TRACE
    tft.color565(5, 5, 5), // GRAPH_BACKGROUND
    TFT_GREEN,             // ENABLED
    tft.color565(5, 5, 5)  // DISABLED
};

uint16_t *colors = darkMode;

void initGraph()
{
  // Graph area is 280 pixels wide, 150 high, dark grey background
  gr.createGraph(235, 150, tft.color565(5, 5, 5));

  // x scale units is from 0 to 100, y scale units is -50 to 50
  gr.setGraphScale(0, MAX_GRAPH_FREQUENCY + 1, MIN_GRAPH_AMPLITUDE, MAX_GRAPH_AMPLITUDE);

  // X grid starts at 0 with lines every 10 x-scale units
  // Y grid starts at -50 with lines every 25 y-scale units
  // blue grid
  gr.setGraphGrid(0, 4000, MIN_GRAPH_AMPLITUDE, 5.0, colors[COL_GRID]);

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

void updateUserSettings(int filterType, int filterIndex)
{
  // Update filter types
  tft.setTextSize(2);
  for (int i = 0; i < sizeof(filterTypes) / sizeof(filterTypes[0]); i++)
  {
    if (i == filterType)
    {
      tft.setTextColor(colors[COL_BACKGROUND], colors[COL_ENABLED]);
    }
    else
    {
      tft.setTextColor(colors[COL_BACKGROUND], colors[COL_DISABLED]);
    }
    tft.setCursor(10 + i * 55, 210);
    tft.print(filterTypes[i]);
  }

  // Update filter index
  for (int i = 0; i < NUM_FILTERS; i++)
  {
    if (i == filterIndex)
    {
      tft.setTextColor(colors[COL_BACKGROUND], colors[COL_ENABLED]);
    }
    else
    {
      tft.setTextColor(colors[COL_BACKGROUND], colors[COL_DISABLED]);
    }
    tft.setCursor(178 + i * 50, 210);
    tft.printf(" %d ", i + 1);
  }
}

void updateGraph(float *x, float *y, int length)
{
  // Clear the previous graph by redrawing the background
  gr.drawGraph(40, 10);
  tr.startTrace(TFT_CYAN);

  // Add the new line to the graph in yellow
  for (int i = 0; i < length - 1; i++)
  {
    tr.addPoint(x[i], y[i]);
  }
}

float getBiquadGain(float f, float fs, float b0, float b1, float b2, float a1, float a2)
{
  float w = 2.0 * M_PI * f / fs;
  std::complex<float> jw(0, w);
  std::complex<float> numerator = b0 + b1 * exp(-jw) + b2 * exp(-2.0f * jw);
  std::complex<float> denominator = 1.0f + a1 * exp(-jw) + a2 * exp(-2.0f * jw);
  return log10(std::abs(numerator / denominator)) * 20.0f;
}

void updateFilterCoeffs(Packet *packet)
{
  const int dataLength = 100;
  float freqResponse[dataLength];
  float freq[dataLength];
  const float maxFreq = 20000.0f;

  // Extract filter coefficients into variables
  for(int i = 0; i< FILTER_BANDS; i++)
  {
    float b0 = ntohf(packet->data.coeffs[i].b0);
    float b1 = ntohf(packet->data.coeffs[i].b1);
    float b2 = ntohf(packet->data.coeffs[i].b2);
    float a1 = ntohf(packet->data.coeffs[i].a1);
    float a2 = ntohf(packet->data.coeffs[i].a2);
    Serial.printf("Band %d Coefficients: b0=0x%08X, b1=0x%08X, b2=0x%08X, a1=0x%08X, a2=0x%08X\n", i, b0, b1, b2, a1, a2);
    }

  for (int i = 1; i < dataLength; i++)
  {
    freq[i] = float(i) / float(dataLength) * maxFreq;

    // Initialize frequency response
    if(packet->displayMode == DISPLAY_MODE_COMBINED)
    {
      freqResponse[i] = 0.0f;
      for(int j = 0; j < FILTER_BANDS; j++)
      {
        float b0 = ntohf(packet->data.coeffs[j].b0);
        float b1 = ntohf(packet->data.coeffs[j].b1);
        float b2 = ntohf(packet->data.coeffs[j].b2);
        float a1 = ntohf(packet->data.coeffs[j].a1);
        float a2 = ntohf(packet->data.coeffs[j].a2);  
        freqResponse[i] += getBiquadGain(freq[i], SAMPLE_FREQ, b0, b1, b2, a1, a2);
      }
    }
    else
    {
      int index = packet->selectedFilterBand;
      float b0 = ntohf(packet->data.coeffs[index].b0);
      float b1 = ntohf(packet->data.coeffs[index].b1);
      float b2 = ntohf(packet->data.coeffs[index].b2);
      float a1 = ntohf(packet->data.coeffs[index].a1);
      float a2 = ntohf(packet->data.coeffs[index].a2);  
      freqResponse[i] = getBiquadGain(freq[i], SAMPLE_FREQ, b0, b1, b2, a1, a2);
    }
  }
  updateGraph(freq, freqResponse, dataLength);
}

void updateFilterParameters(Packet *packet)
{
  // Extract set gain, Q and frequency
  int index = packet->selectedFilterBand;
  int filterType = int(packet->data.params[index].filterType);
  float gain = ntohf(packet->data.params[index].gain);
  float Q = ntohf(packet->data.params[index].Q);
  float frequency = ntohf(packet->data.params[index].frequency);
  tft.setTextColor(TFT_WHITE, TFT_BLACK); // White text with black background
  tft.setTextSize(1);
  tft.setCursor(10, 180);
  tft.printf("Gain: %.2f dB  ", gain);
  tft.setCursor(120, 180);
  tft.printf("Q: %.2f", Q);
  tft.setCursor(190, 180);
  tft.printf("Freq: %.2f Hz  ", frequency);
  updateUserSettings(filterType, int(index));
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
  updateUserSettings(0, 0);

  // Initialize communication with the DSP
  Wire.setPins(32, 25);
  Wire.setBufferSize(1024);
  Wire.onReceive(processIncomingPacket);
  Wire.begin(0xb1ce);
}

void loop()
{
  Packet *packet = popPacket();
  if (packet == NULL)
  {
    return;
  }
  switch (packet->packetType)
  {
  case PACKET_COEFFS: 
    updateFilterCoeffs(packet);
    break;
  case PACKET_PARAMS: 
    updateFilterParameters(packet);
    break;
  default:
    Serial.println("Unknown packet type received");
    return;
  }
}
