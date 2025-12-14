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

// Touchscreen pins
#define XPT2046_IRQ 36  // T_IRQ
#define XPT2046_MOSI 32 // T_DIN
#define XPT2046_MISO 39 // T_OUT
#define XPT2046_CLK 25  // T_CLK
#define XPT2046_CS 33   // T_CS

/*
#define XPT2046_IRQ 4   // T_IRQ
#define XPT2046_MOSI 16  // T_DIN
#define XPT2046_MISO 14  // T_OUT
#define XPT2046_CLK 13   // T_CLK
#define XPT2046_CS 9    // T_CS
*/

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
}

// Compute the frequency response of a biquad filter
// length: Number of frequency points
// fNyq: Nyquist frequency
// a1, a2: Denomiinator coefficients
// b0, b1, b2: Numerator coefficients
void frequencyResponseBiquad(int length, int fs, float a1, float a2, float b0, float b1, float b2, float *response)
{
  for (int i = 0; i < length; i++)
  {
    float w = 2.0 * M_PI * (float(i) / float(length));
    std::complex<float> jw(0, w);
    std::complex<float> numerator = b0 + b1 * exp(-jw) + b2 * exp(-2.0f * jw);
    std::complex<float> denominator = 1.0f + a1 * exp(-jw) + a2 * exp(-2.0f * jw);
    std::complex<float> H = numerator / denominator;
    Serial.print("H[");
    Serial.print(w);
    Serial.print("] = ");
    Serial.println(std::abs(H));
    response[i] = log10(std::abs(H)) * 20.0f;
  }
}

float getBiquadGain(float f, float fs, float a1, float a2, float b0, float b1, float b2)
{
  float w = 2.0 * M_PI * f / fs;
  std::complex<float> jw(0, w);
  std::complex<float> numerator = b0 + b1 * exp(-jw) + b2 * exp(-2.0f * jw);
  std::complex<float> denominator = 1.0f + a1 * exp(-jw) + a2 * exp(-2.0f * jw);
  return log10(std::abs(numerator / denominator)) * 20.0f;
}

float phase = 0.0;

void loop()
{
  // Your drawing/interaction code here
  // Example data for the graph
  const int dataLength = 100;

  float b0 = 1.058592344432e+00;
  float b1 = -1.566598405748e+00;
  float b2 = 9.160617298456e-01;
  float a1 = -1.566598405748e+00;
  float a2 = 9.746540742780e-01;

  float freqResponse[dataLength];
  float freq[dataLength];
  const float maxFreq = 20000.0f;
  for (int i = 0; i < dataLength; i++)
  {
    freq[i] = float(i) / float(dataLength) * maxFreq;
    freqResponse[i] = getBiquadGain(freq[i], 44100.0f, a1, a2, b0, b1, b2);
  }
  updateGraph(freq, freqResponse, dataLength);

  // Print frequency response data to Serial Monitor
  for (int i = 0; i < dataLength; i++)
  {
    Serial.print("Frequency: ");
    Serial.print(freq[i]); // Frequency in Hz
    Serial.print(" Hz, Magnitude: ");
    Serial.print(freqResponse[i]); // Magnitude in dB
    Serial.println(" dB");
  }
  delay(1000);
}
