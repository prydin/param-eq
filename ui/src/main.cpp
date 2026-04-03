#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <complex>
#include <Wire.h>

#include "netconv.h"
#include "registers.h"
#include "ui_lvgl.h"
#include "../../common/filter.h"
#include "../../common/constants.h"

TFT_eSPI tft = TFT_eSPI(); // Create tft object

#define MAX_GRAPH_FREQUENCY 20000.0 // Maximum frequency for graph x axis
#define MIN_GRAPH_FREQUENCY 2.0     // Minimum frequency for graph x axis

#define SAMPLE_FREQ 44100.0f
#define RESPONSE_POINTS 240
#ifndef UI_LIGHT_MODE
#define UI_LIGHT_MODE 0
#endif
#ifndef UI_ENABLE_DEMO
#define UI_ENABLE_DEMO 0
#endif

#if UI_ENABLE_DEMO
#define UI_DEMO_INTERVAL_MS 500U
#define UI_DEMO_IDLE_TIMEOUT_MS 1000U
#endif

RegisterBank registers;

void unpackVuMeterLevels(uint32_t packedLevel, uint16_t &leftLevel, uint16_t &rightLevel)
{
  leftLevel = static_cast<uint16_t>(packedLevel >> 16);
  rightLevel = static_cast<uint16_t>(packedLevel & 0xFFFFU);
}

#if UI_ENABLE_DEMO
struct DemoState {
  uint8_t step;
  uint32_t lastUpdateMs;
  uint32_t lastRealUpdateMs;
};
#endif

inline float getBiquadGain(float f, float fs, float b0, float b1, float b2, float a1, float a2)
{
  float w = 2.0f * PI * f / fs;
  std::complex<float> jw(0.0f, w);
  std::complex<float> numerator = b0 + b1 * exp(-jw) + b2 * exp(-2.0f * jw);
  std::complex<float> denominator = 1.0f + a1 * exp(-jw) + a2 * exp(-2.0f * jw);
  return log10f(std::abs(numerator / denominator)) * 20.0f;
}

void computeResponses(RegisterBank *registers, float *selectedResponse, float *combinedResponse)
{
  float logStep = (log10f(MAX_GRAPH_FREQUENCY) - log10f(MIN_GRAPH_FREQUENCY)) / float(RESPONSE_POINTS - 1);
  float logMinFreq = log10f(MIN_GRAPH_FREQUENCY);
  int index = registers->getFilterSelect();
  float b0 = registers->getFilterCoeff(index, 0);
  float b1 = registers->getFilterCoeff(index, 1);
  float b2 = registers->getFilterCoeff(index, 2);
  float a1 = registers->getFilterCoeff(index, 3);
  float a2 = registers->getFilterCoeff(index, 4);

  for (int i = 0; i < RESPONSE_POINTS; i++)
  {
    float f = exp10f(i * logStep + logMinFreq);
    selectedResponse[i] = getBiquadGain(f, SAMPLE_FREQ, b0, b1, b2, a1, a2);
    combinedResponse[i] = selectedResponse[i];
  }

  if (registers->getDisplayMode() != DISPLAY_MODE_COMBINED)
  {
    return;
  }

  for (int i = 0; i < RESPONSE_POINTS; i++)
  {
    float f = exp10f(i * logStep + logMinFreq);
    combinedResponse[i] = 0.0f;
    for (int j = 0; j < FILTER_BANDS; j++)
    {
      float cb0 = registers->getFilterCoeff(j, 0);
      float cb1 = registers->getFilterCoeff(j, 1);
      float cb2 = registers->getFilterCoeff(j, 2);
      float ca1 = registers->getFilterCoeff(j, 3);
      float ca2 = registers->getFilterCoeff(j, 4);

      // Ignore bands that have not received valid coefficients yet.
      if (cb0 == 0.0f && cb1 == 0.0f && cb2 == 0.0f && ca1 == 0.0f && ca2 == 0.0f)
      {
        continue;
      }

      combinedResponse[i] += getBiquadGain(f, SAMPLE_FREQ, cb0, cb1, cb2, ca1, ca2);
    }
  }

  float masterGain = registers->getInputGain();
  for (int i = 0; i < RESPONSE_POINTS; i++)
  {
    combinedResponse[i] += masterGain;
  }
}

#if UI_ENABLE_DEMO
void computeDemoResponses(const UiData &data, float *selectedResponse, float *combinedResponse)
{
  const float logStep = (log10f(MAX_GRAPH_FREQUENCY) - log10f(MIN_GRAPH_FREQUENCY)) / float(RESPONSE_POINTS - 1);
  const float logMinFreq = log10f(MIN_GRAPH_FREQUENCY);
  const float center = log10f(fmaxf(data.frequency, MIN_GRAPH_FREQUENCY));
  const float width = 0.22f + 0.03f * data.filterIndex;

  for (int i = 0; i < RESPONSE_POINTS; i++)
  {
    const float frequency = exp10f(i * logStep + logMinFreq);
    const float distance = (log10f(frequency) - center) / width;
    const float bell = expf(-0.5f * distance * distance);

    if (data.filterType == LOWSHELF)
    {
      const float shelfMix = 1.0f / (1.0f + expf((log10f(frequency) - center) * 7.0f));
      selectedResponse[i] = data.filterGain * shelfMix;
    }
    else if (data.filterType == HIGHSHELF)
    {
      const float shelfMix = 1.0f / (1.0f + expf((center - log10f(frequency)) * 7.0f));
      selectedResponse[i] = data.filterGain * shelfMix;
    }
    else
    {
      selectedResponse[i] = data.filterGain * bell;
    }

    combinedResponse[i] = selectedResponse[i];
    if (data.displayMode == DISPLAY_MODE_COMBINED)
    {
      combinedResponse[i] += data.inputGain;
      combinedResponse[i] += 1.4f * sinf((float(i) / RESPONSE_POINTS) * 2.0f * PI + data.filterIndex);
    }
  }
}

UiData makeDemoData(uint8_t step)
{
  static const float frequencies[] = {45.0f, 120.0f, 350.0f, 1000.0f, 2800.0f, 8200.0f};
  static const float filterGains[] = {-6.0f, -3.0f, 0.0f, 3.0f, 6.0f, 9.0f};
  static const float qs[] = {0.7f, 1.0f, 1.4f, 2.0f, 2.8f, 4.0f};
  static const float inputGains[] = {-3.0f, -1.5f, 0.0f, 1.5f, 3.0f, 4.5f};
  static const float outputGains[] = {-12.0f, -9.0f, -6.0f, -3.0f, 0.0f, 1.5f};

  const size_t idx = step % 6;

  UiData data = {};
  data.filterType = step % 3;
  data.filterIndex = step % FILTER_BANDS;
  data.userInput = 1;
  data.displayMode = (step % 4 >= 2) ? DISPLAY_MODE_COMBINED : DISPLAY_MODE_INDIVIDUAL;
  data.sampleRate = 44100;
  data.inputGain = inputGains[idx];
  data.filterGain = filterGains[idx];
  data.q = qs[idx];
  data.outputGain = outputGains[idx];
  data.frequency = frequencies[idx];
  return data;
}
#endif

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

  const int payloadBytes = numBytes - 1;
  if (payloadBytes != 4)
  {
    while (Wire.available())
    {
      Wire.read();
    }
    return;
  }

  // Read the data
  uint32_t data = 0;
  uint8_t* ptr = reinterpret_cast<uint8_t*>(&data);
  size_t bytesRead = 0;
  while (Wire.available() && bytesRead < sizeof(data))
  {
    ptr[bytesRead++] = Wire.read();
    // Serial.printf("Read byte: %02x\n", ptr[bytesRead - 1]);
  }

  if (bytesRead != sizeof(data))
  {
    return;
  }

  registers.updateRegister(registerNumber, data);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting LVGL UI...");
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

  tft.fillScreen(UI_LIGHT_MODE ? TFT_WHITE : TFT_BLACK);
  ui_lvgl_init(tft);

  // Initialize communication with the DSP
  Wire.setPins(32, 25);
  Wire.setBufferSize(1024);
  Wire.onReceive(processI2C);
  Wire.begin(DISPLAY_I2C_ADDRESS);
}

void loop()
{
#if UI_ENABLE_DEMO
  static DemoState demoState = {0, 0, 0};
#endif

  ui_lvgl_task();

  if (registers.isReady())
  {
    float selectedResponse[RESPONSE_POINTS] = {};
    float combinedResponse[RESPONSE_POINTS] = {};
    computeResponses(&registers, selectedResponse, combinedResponse);

    UiData data = {};
    data.filterType = registers.getFilterType();
    data.filterIndex = registers.getFilterSelect();
    data.userInput = registers.getUserInput();
    data.displayMode = registers.getDisplayMode();
    data.uiMode = registers.getUiMode();

    if (!registers.takeVuMeterUpdate(data.vuLeft, data.vuRight))
    {
      unpackVuMeterLevels(registers.getVuMeterPacked(), data.vuLeft, data.vuRight);
    }

    registers.copyFftBins(data.fftLeft, 16);
    uint8_t fftAccum[16] = {};
    if (registers.takeFftUpdate(fftAccum, 16))
    {
      for (int i = 0; i < 16; i++)
      {
        data.fftLeft[i] = fftAccum[i];
      }
    }

    data.sampleRate = registers.getSampleRate();
    data.inputGain = registers.getInputGain();
    data.filterGain = registers.getFilterGain();
    data.q = registers.getQ();
    data.outputGain = registers.getOutputGain();
    data.frequency = registers.getFrequency();

    ui_lvgl_update(data, selectedResponse, combinedResponse, RESPONSE_POINTS);

#if UI_ENABLE_DEMO
    demoState.lastRealUpdateMs = millis();
#endif
    return;
  }

  uint16_t vuLeft = 0;
  uint16_t vuRight = 0;
  if (registers.takeVuMeterUpdate(vuLeft, vuRight))
  {
    ui_lvgl_update_vu(vuLeft, vuRight);
  }

#if UI_ENABLE_DEMO
  const uint32_t now = millis();
  if (now - demoState.lastRealUpdateMs < UI_DEMO_IDLE_TIMEOUT_MS)
  {
    return;
  }

  if (now - demoState.lastUpdateMs >= UI_DEMO_INTERVAL_MS)
  {
    float selectedResponse[RESPONSE_POINTS] = {};
    float combinedResponse[RESPONSE_POINTS] = {};
    UiData demoData = makeDemoData(demoState.step++);
    computeDemoResponses(demoData, selectedResponse, combinedResponse);
    ui_lvgl_update(demoData, selectedResponse, combinedResponse, RESPONSE_POINTS);
    demoState.lastUpdateMs = now;
  }
#endif
}
