#include <SPI.h>
#include <TFT_eSPI.h>
#include <TFT_eWidget.h>
#include <esp_mac.h>

// Install the "XPT2046_Touchscreen" library by Paul Stoffregen to use the Touchscreen - https://github.com/PaulStoffregen/XPT2046_Touchscreen
// Note: this library doesn't require further configuration
#define ST7796_DRIVER
#include <XPT2046_Touchscreen.h>

#include <complex.h>
#include <Wire.h>

#include "netconv.h"
#include "registers.h"
#include "../../common/filter.h"
#include "../../common/constants.h"

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
#define MIN_GRAPH_FREQUENCY 2.0     // Minimum frequency for graph x axis
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
#define COL_ALERT 8

#define NUM_COLORS 9

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
    TFT_GREY,              // DISABLED
    TFT_RED                // ALERT
};

uint16_t lightMode[NUM_COLORS] = {
    TFT_WHITE,             // BACKGROUND
    TFT_WHITE,             // TEXT_BACKGROUND
    TFT_BLACK,             // FOREGROUND
    TFT_DARKGREEN,         // GRID
    TFT_YELLOW,            // TRACE
    tft.color565(5, 5, 5), // GRAPH_BACKGROUND
    TFT_GREEN,             // ENABLED
    tft.color565(5, 5, 5), // DISABLED
    TFT_RED                // ALERT
};

uint16_t *colors = darkMode;

RegisterBank registers;

void initGraph()
{
  // Graph area is 280 pixels wide, 150 high, dark grey background
  gr.createGraph(235, 160, tft.color565(5, 5, 5));

  // x scale units is from 0 to 100, y scale units is -50 to 50
  gr.setGraphScale(log10(MIN_GRAPH_FREQUENCY), log10(MAX_GRAPH_FREQUENCY), MIN_GRAPH_AMPLITUDE, MAX_GRAPH_AMPLITUDE);

  // X grid starts at 0 with lines every 10 x-scale units
  // Y grid starts at -50 with lines every 25 y-scale units
  // blue grid
  gr.setGraphGrid(log10(MIN_GRAPH_FREQUENCY), 1, MIN_GRAPH_AMPLITUDE, 5.0, colors[COL_GRID]);

  // Draw the x axis scale
  tft.setTextDatum(TC_DATUM); // Top centre text datum
  tft.drawNumber(2, gr.getPointX(log10(2)) + 43, gr.getPointY(-15.0) + 13);
  tft.drawNumber(20, gr.getPointX(log10(20)) + 43, gr.getPointY(-15.0) + 13);
  tft.drawNumber(200, gr.getPointX(log10(200)) + 43, gr.getPointY(-15.0) + 13);
  tft.drawNumber(2000, gr.getPointX(log10(2000)) + 43, gr.getPointY(-15.0) + 13);
  tft.drawNumber(20000, gr.getPointX(log10(20000)) + 43, gr.getPointY(-15.0) + 13);
  // Draw the y axis scale
  tft.setTextDatum(MR_DATUM); // Middle right text datum
  tft.drawNumber(-15, gr.getPointX(0.0) + 32, gr.getPointY(-15.0) + 13);
  tft.drawNumber(0, gr.getPointX(0.0) + 32, gr.getPointY(0.0) + 13);
  tft.drawNumber(15, gr.getPointX(0.0) + 32, gr.getPointY(15.0) + 13);

  // Draw empty graph, top left corner at 40,10 on TFT
  gr.drawGraph(40, 10);

  // Drav information box to the right of the graph
  tft.setTextDatum(TL_DATUM); // Top left text datum
  tft.drawRect(280, 28, 40, 142, colors[COL_FOREGROUND]);
  tft.drawString("Fs", 285, 32);
  tft.drawString("MGain", 285, 61);
  tft.drawString("FGain", 285, 89); 
  tft.drawString("Q", 285, 117);
  tft.drawString("Vol", 285, 145);
}

void updateUserSettings(int filterType, int filterIndex, int displayMode)
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

  // Update dislay mode
  if (displayMode == DISPLAY_MODE_COMBINED)
  {
    tft.setTextColor(colors[COL_BACKGROUND], colors[COL_ENABLED]);
  }
  else
  {
    tft.setTextColor(colors[COL_BACKGROUND], colors[COL_DISABLED]);
  }

  tft.setCursor(280, 10);
  tft.print("SUM");
}

void updateClipAlert(bool clipped)
{
  tft.setTextSize(2);
  // Update dislay mode
  if (clipped)
  {
    tft.setTextColor(colors[COL_BACKGROUND], colors[COL_ALERT]);
  }
  else
  {
    tft.setTextColor(colors[COL_BACKGROUND], colors[COL_DISABLED]);
  }

  tft.setCursor(280, 10);
  tft.print("CLP");
}

void updateGraph(float *x, float *y, int length, uint16_t color, bool clearPrevious)
{
  // Clear the previous graph by redrawing the background
  if (clearPrevious)
  {
    gr.drawGraph(40, 10);
  }
  tr.startTrace(color);

  // Add the new line to the graph in yellow
  for (int i = 0; i < length - 1; i++)
  {
    float f = x[i];
    if (f > SAMPLE_FREQ / 2.0f)
    {
      break;
    }
    tr.addPoint(log10(f), y[i]);
  }
}

inline float getBiquadGain(float f, float fs, float b0, float b1, float b2, float a1, float a2)
{
  float w = 2.0 * M_PI * f / fs;
  std::complex<float> jw(0, w);
  std::complex<float> numerator = b0 + b1 * exp(-jw) + b2 * exp(-2.0f * jw);
  std::complex<float> denominator = 1.0f + a1 * exp(-jw) + a2 * exp(-2.0f * jw);
  return log10(std::abs(numerator / denominator)) * 20.0f;
}

void updateFilterCoeffs(RegisterBank *registers)
{
  const int dataLength = 100;
  float freqResponse[dataLength];
  float freq[dataLength];

  float logStep = (log10(MAX_GRAPH_FREQUENCY) - log10(MIN_GRAPH_FREQUENCY)) / float(dataLength - 1);
  float logMinFreq = log10(MIN_GRAPH_FREQUENCY);
  int index = registers->getFilterSelect();
  float b0 = registers->getFilterCoeff(index, 0);
  float b1 = registers->getFilterCoeff(index, 1);
  float b2 = registers->getFilterCoeff(index, 2);
  float a1 = registers->getFilterCoeff(index, 3);
  float a2 = registers->getFilterCoeff(index, 4);
  // Print coefficients
  //Serial.printf("Filter %d Coefficients in updateFilterCoeffs: %f, %f, %f, %f, %f\n", index, b0, b1, b2, a1, a2);

  for (int i = 0; i < dataLength; i++)
  {
    freq[i] = exp10(i * logStep + logMinFreq);
    freqResponse[i] = getBiquadGain(freq[i], SAMPLE_FREQ, b0, b1, b2, a1, a2);
  }
  updateGraph(freq, freqResponse, dataLength, TFT_CYAN, true);

  if (registers->getDisplayMode() == DISPLAY_MODE_COMBINED)
  {
    for (int i = 0; i < dataLength; i++)
    {
      freq[i] = exp10(i * logStep + logMinFreq);

      freqResponse[i] = 0.0f;
      for (int j = 0; j < FILTER_BANDS; j++)
      {
        float b0 = registers->getFilterCoeff(j, 0);
        float b1 = registers->getFilterCoeff(j, 1);
        float b2 = registers->getFilterCoeff(j, 2);
        float a1 = registers->getFilterCoeff(j, 3);
        float a2 = registers->getFilterCoeff(j, 4);
        freqResponse[i] += getBiquadGain(freq[i], SAMPLE_FREQ, b0, b1, b2, a1, a2);
      }
    }
    // Add master gain
    float masterGain = registers->getInputGain();
    for (int i = 0; i < dataLength; i++)
    {
      freqResponse[i] += masterGain;
    }
    updateGraph(freq, freqResponse, dataLength, TFT_LIGHTGREY, false);
  }
}

void updateFilterParameters(RegisterBank *registers)
{
  // Extract set gain, Q and frequency
  int index = registers->getFilterSelect();
  int filterType = int(registers->getFilterType());
  float gain = registers->getFilterGain();
  float Q = registers->getQ();
  float frequency = registers->getFrequency();
  tft.setTextColor(TFT_WHITE, TFT_BLACK); // White text with black background
  tft.setTextSize(1);
  /*  tft.setCursor(10, 180);
    tft.printf("Gain: %.2f dB  ", gain);
    tft.setCursor(120, 180);
    tft.printf("Q: %.2f", Q); */

  // Draw the frequency marker on the graph
  tft.fillRect(40, 182, 320 - 40, 22, colors[COL_TEXT_BACKGROUND]); // Clear area
  tft.setTextDatum(TC_DATUM);                                       // Top centre text datum
  tft.setCursor(gr.getPointX(log10(frequency)), 182);
  tft.printf("^");
  tft.setCursor(gr.getPointX(log10(frequency)) - 5, 190);
  tft.printf("%.0f", frequency);

  tft.setTextDatum(TL_DATUM);
  tft.setCursor(285, 42);
  Serial.printf("Sample Rate in updateFilterParameters: %d\n", registers->getSampleRate());
  if(registers->getSampleRate() == 0) {
    tft.print(" ----");
  } else {
    tft.printf("%5.1f", registers->getSampleRate() / 1000.0f);
  }
  tft.setCursor(285, 74);
  tft.printf("%5.1f", registers->getInputGain());
  tft.setCursor(285, 102);
  tft.printf("%5.1f", gain);
  tft.setCursor(285, 130);
  tft.printf("%5.1f", Q);
  tft.setCursor(285, 158);
  if(registers->getOutputGain() <= -100.0f) {
    tft.printf("%5.0f", registers->getOutputGain());
  } else {
    tft.printf("%5.1f", registers->getOutputGain());
  }

  updateUserSettings(filterType, int(index), registers->getDisplayMode());
}

void processI2C(int numBytes)
{
  // Serial.printf("I2C data received: %d bytes\n", numBytes);
  if (numBytes > 5)
  {
    Serial.printf("I2C buffer overflow: %d bytes received\n", numBytes);
    return;
  }
  // Read register number
  uint8_t registerNumber = Wire.read();

  // Read the data
  uint32_t data;
  uint8_t* ptr = reinterpret_cast<uint8_t*>(&data);
  size_t bytesRead = 0;
  while (Wire.available() && bytesRead < numBytes)
  {
    ptr[bytesRead++] = Wire.read();
    // Serial.printf("Read byte: %02x\n", ptr[bytesRead - 1]);
  }
  registers.updateRegister(registerNumber, data);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting UI...");
  tft.init();
  tft.setRotation(3);

#ifdef ST7796_DRIVER
  // Deal with cheap ST7796 displays that have incorrect MADCTL settings
  // tft.setRotation(3);
  tft.begin_nin_write();
  tft.writecommand(ST7796_MADCTL);
  tft.writedata(ST7796_MADCTL_MV | ST7796_MADCTL_MX | ST7796_MADCTL_MH | ST7796_MADCTL_BGR);
  tft.end_nin_write();
  tft.invertDisplay(true);
#endif

  tft.fillScreen(TFT_BLACK); // Clear screen to black
  // tft.setCursor(20, 20);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  initGraph();
  updateUserSettings(0, 0, DISPLAY_MODE_INDIVIDUAL);

  // Initialize communication with the DSP
  Wire.setPins(32, 25);
  Wire.setBufferSize(1024);
  Wire.onReceive(processI2C);
  Wire.begin(DISPLAY_I2C_ADDRESS);
}

void loop()
{
  static float masterGain = 0.0f;
  if (registers.isReady())
  {
    updateFilterCoeffs(&registers);
    updateFilterParameters(&registers);
  }
  /*
  Packet *packet = popPacket();
  if (packet == NULL)
  {
    return;
  }
  switch (packet->packetType)
  {
  case PACKET_COEFFS:
    updateFilterCoeffs(packet);
    masterGain = ntohf(packet->data.filters.masterGain);
    break;
  case PACKET_PARAMS:
    updateFilterParameters(packet, masterGain);
    break;
  case PACKET_CLIP_ALERT:
    updateClipAlert(packet->data.clipAlert.clipped);
    if (packet->data.clipAlert.clipped)
    {
      Serial.println("Clipping detected!");
    }
    break;
  default:
    Serial.println("Unknown packet type received");
    return;
  } */
}
