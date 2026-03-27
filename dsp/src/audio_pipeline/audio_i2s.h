#pragma once

#include <Arduino.h>
#include <DMAChannel.h>

extern void (*i2sAudioCallback)(int32_t **inputs, int32_t **outputs);

class Timers
{
    friend class AudioOutputI2S;
public:
    static const int TIMER_COUNT = 20;
    static const uint8_t TIMER_TOTAL = 19;
    static float TimeAvg[TIMER_COUNT];
    static float TimePeak[TIMER_COUNT];
    static float TimeMax[TIMER_COUNT];

    static void Lap(uint8_t timerIndex);
    static float GetAvg(uint8_t timerIndex = 0);
    static float GetPeak(uint8_t timerIndex = 0);
    static float GetMax(uint8_t timerIndex = 0);
    static void Clear(uint8_t timerIndex = 0);
    static float GetAvgPeriod();
    static float GetCpuLoad();

private:
    static int TimeFrameStart;
    static float TimeFramePeriod;
    static void ResetFrame();
    static void LapInner(uint8_t timerIndex);
};

class AudioOutputI2S
{
public:
    static bool Enabled;

    AudioOutputI2S() = default;
    void begin();
    void setSampleRate(uint32_t sampleRate);
    uint32_t getSampleRate() const { return sampleRateHz; }

private:
    static void configI2S();
    static void applyClockConfig(uint32_t sampleRate, bool force = false);
    static bool isConfigured;
    static DMAChannel dma;
    static uint32_t sampleRateHz;
    static void isr();
};
