#include "audio_i2s.h"

#include <Arduino.h>
#include <imxrt.h>
#include <cmath>

#include "audio_config.h"

namespace
{
void audioCallbackSilence(int32_t **, int32_t **outputs)
{
    for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; ++i)
    {
        outputs[0][i] = 0;
        outputs[1][i] = 0;
    }
}

FLASHMEM void setAudioClock(int nfact, int32_t nmult, uint32_t ndiv, bool force)
{
    if (!force && (CCM_ANALOG_PLL_AUDIO & CCM_ANALOG_PLL_AUDIO_ENABLE))
    {
        return;
    }

    CCM_ANALOG_PLL_AUDIO = CCM_ANALOG_PLL_AUDIO_BYPASS | CCM_ANALOG_PLL_AUDIO_ENABLE |
                           CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(2) |
                           CCM_ANALOG_PLL_AUDIO_DIV_SELECT(nfact);

    CCM_ANALOG_PLL_AUDIO_NUM = nmult & CCM_ANALOG_PLL_AUDIO_NUM_MASK;
    CCM_ANALOG_PLL_AUDIO_DENOM = ndiv & CCM_ANALOG_PLL_AUDIO_DENOM_MASK;

    CCM_ANALOG_PLL_AUDIO &= ~CCM_ANALOG_PLL_AUDIO_POWERDOWN;
    while (!(CCM_ANALOG_PLL_AUDIO & CCM_ANALOG_PLL_AUDIO_LOCK))
    {
    }

    const int divPostPll = 1;
    CCM_ANALOG_MISC2 &= ~(CCM_ANALOG_MISC2_DIV_MSB | CCM_ANALOG_MISC2_DIV_LSB);
    if (divPostPll > 1)
    {
        CCM_ANALOG_MISC2 |= CCM_ANALOG_MISC2_DIV_LSB;
    }
    if (divPostPll > 3)
    {
        CCM_ANALOG_MISC2 |= CCM_ANALOG_MISC2_DIV_MSB;
    }

    CCM_ANALOG_PLL_AUDIO &= ~CCM_ANALOG_PLL_AUDIO_BYPASS;
}
} // namespace

float Timers::TimeAvg[Timers::TIMER_COUNT];
float Timers::TimePeak[Timers::TIMER_COUNT];
float Timers::TimeMax[Timers::TIMER_COUNT];
int Timers::TimeFrameStart = 0;
float Timers::TimeFramePeriod = 1333.33f;

void Timers::Lap(uint8_t timerIndex)
{
    if (timerIndex >= TIMER_TOTAL)
    {
        return;
    }
    LapInner(timerIndex);
}

void Timers::LapInner(uint8_t timerIndex)
{
    if (timerIndex >= TIMER_COUNT)
    {
        return;
    }

    int value = float(micros() - TimeFrameStart);
    TimeAvg[timerIndex] = 0.995f * TimeAvg[timerIndex] + 0.005f * value;
    TimePeak[timerIndex] = 0.995f * TimePeak[timerIndex];
    if (value > TimePeak[timerIndex])
    {
        TimePeak[timerIndex] = value;
    }
    if (value > TimeMax[timerIndex])
    {
        TimeMax[timerIndex] = value;
    }
}

float Timers::GetAvg(uint8_t timerIndex)
{
    if (timerIndex >= TIMER_COUNT)
    {
        return -1;
    }
    return TimeAvg[timerIndex];
}

float Timers::GetPeak(uint8_t timerIndex)
{
    if (timerIndex >= TIMER_COUNT)
    {
        return -1;
    }
    return TimePeak[timerIndex];
}

float Timers::GetMax(uint8_t timerIndex)
{
    if (timerIndex >= TIMER_COUNT)
    {
        return -1;
    }
    return TimeMax[timerIndex];
}

void Timers::Clear(uint8_t timerIndex)
{
    if (timerIndex >= TIMER_COUNT)
    {
        return;
    }
    TimeAvg[timerIndex] = 0;
    TimePeak[timerIndex] = 0;
    TimeMax[timerIndex] = 0;
}

float Timers::GetAvgPeriod()
{
    return TimeFramePeriod;
}

float Timers::GetCpuLoad()
{
    return TimePeak[Timers::TIMER_TOTAL] / TimeFramePeriod;
}

void Timers::ResetFrame()
{
    if (TimeFrameStart == 0)
    {
        TimeFrameStart = micros();
        return;
    }

    int oldStart = TimeFrameStart;
    TimeFrameStart = micros();
    TimeFramePeriod = 0.995f * TimeFramePeriod + 0.005f * (TimeFrameStart - oldStart);
}

DMAChannel AudioOutputI2S::dma(false);
bool AudioOutputI2S::Enabled = false;
bool AudioOutputI2S::isConfigured = false;
uint32_t AudioOutputI2S::sampleRateHz = AUDIO_SAMPLE_RATE_DEFAULT;
void (*i2sAudioCallback)(int32_t **inputs, int32_t **outputs) = audioCallbackSilence;

namespace
{
int32_t dataLA[AUDIO_BLOCK_SAMPLES] = {0};
int32_t dataLB[AUDIO_BLOCK_SAMPLES] = {0};
int32_t dataRA[AUDIO_BLOCK_SAMPLES] = {0};
int32_t dataRB[AUDIO_BLOCK_SAMPLES] = {0};
DMAMEM __attribute__((aligned(32))) uint64_t i2sTxBuffer[AUDIO_BLOCK_SAMPLES * 2];
} // namespace

void AudioOutputI2S::begin()
{
    Enabled = true;
    dma.begin(true);
    configI2S();

    CORE_PIN7_CONFIG = 3;
    dma.TCD->SADDR = i2sTxBuffer;
    dma.TCD->SOFF = 4;
    dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);
    dma.TCD->NBYTES_MLNO = 4;
    dma.TCD->SLAST = -sizeof(i2sTxBuffer);
    dma.TCD->DOFF = 0;
    dma.TCD->CITER_ELINKNO = sizeof(i2sTxBuffer) / 4;
    dma.TCD->DLASTSGA = 0;
    dma.TCD->BITER_ELINKNO = sizeof(i2sTxBuffer) / 4;
    dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;
    dma.TCD->DADDR = (void *)((uint32_t)&I2S1_TDR0 + 0);
    dma.triggerAtHardwareEvent(DMAMUX_SOURCE_SAI1_TX);
    dma.enable();

    I2S1_RCSR |= I2S_RCSR_RE | I2S_RCSR_BCE;
    I2S1_TCSR = I2S_TCSR_TE | I2S_TCSR_BCE | I2S_TCSR_FRDE;
    dma.attachInterrupt(isr);
}

void AudioOutputI2S::setSampleRate(uint32_t sampleRate)
{
    if (sampleRate == 0 || sampleRate == sampleRateHz)
    {
        return;
    }

    __disable_irq();
    uint32_t savedTcsr = I2S1_TCSR;
    uint32_t savedRcsr = I2S1_RCSR;
    I2S1_TCSR = 0;
    I2S1_RCSR = 0;
    applyClockConfig(sampleRate, true);
    I2S1_RCSR = savedRcsr;
    I2S1_TCSR = savedTcsr;
    __enable_irq();
}

void AudioOutputI2S::isr()
{
    int32_t *dest;
    int32_t *transmitBufferL;
    int32_t *transmitBufferR;
    int32_t *fillBuffers[2];
    uint32_t saddr = (uint32_t)(dma.TCD->SADDR);

    dma.clearInterrupt();
    if (saddr < (uint32_t)i2sTxBuffer + sizeof(i2sTxBuffer) / 2)
    {
        dest = (int32_t *)&i2sTxBuffer[AUDIO_BLOCK_SAMPLES];
        transmitBufferL = dataLA;
        transmitBufferR = dataRA;
        fillBuffers[0] = dataLB;
        fillBuffers[1] = dataRB;
    }
    else
    {
        dest = (int32_t *)i2sTxBuffer;
        transmitBufferL = dataLB;
        transmitBufferR = dataRB;
        fillBuffers[0] = dataLA;
        fillBuffers[1] = dataRA;
    }

    for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; ++i)
    {
        dest[2 * i] = transmitBufferL[i];
        dest[2 * i + 1] = transmitBufferR[i];
    }

    arm_dcache_flush_delete(dest, sizeof(i2sTxBuffer) / 2);
    Timers::ResetFrame();

    if (!Enabled)
    {
        for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; ++i)
        {
            fillBuffers[0][i] = 0;
            fillBuffers[1][i] = 0;
        }
    }
    else if (Timers::GetCpuLoad() < 0.98f)
    {
        i2sAudioCallback(nullptr, fillBuffers);
    }

    Timers::LapInner(Timers::TIMER_TOTAL);
}

void AudioOutputI2S::configI2S()
{
    CCM_CCGR5 |= CCM_CCGR5_SAI1(CCM_CCGR_ON);

    if ((I2S1_TCSR & I2S_TCSR_TE) != 0 || (I2S1_RCSR & I2S_RCSR_RE) != 0)
    {
        CORE_PIN23_CONFIG = 3;
        CORE_PIN20_CONFIG = 3;
        return;
    }

    applyClockConfig(sampleRateHz);

    CORE_PIN23_CONFIG = 3;
    CORE_PIN20_CONFIG = 3;
    CORE_PIN21_CONFIG = 3;

    constexpr int rsync = 0;
    constexpr int tsync = 1;

    I2S1_TMR = 0;
    I2S1_TCR1 = I2S_TCR1_RFW(1);
    I2S1_TCR2 = I2S_TCR2_SYNC(tsync) | I2S_TCR2_BCP |
                (I2S_TCR2_BCD | I2S_TCR2_DIV(1) | I2S_TCR2_MSEL(1));
    I2S1_TCR3 = I2S_TCR3_TCE;
    I2S1_TCR4 = I2S_TCR4_FRSZ(1) | I2S_TCR4_SYWD(31) | I2S_TCR4_MF |
                I2S_TCR4_FSD | I2S_TCR4_FSE | I2S_TCR4_FSP;
    I2S1_TCR5 = I2S_TCR5_WNW(31) | I2S_TCR5_W0W(31) | I2S_TCR5_FBT(31);

    I2S1_RMR = 0;
    I2S1_RCR1 = I2S_RCR1_RFW(1);
    I2S1_RCR2 = I2S_RCR2_SYNC(rsync) | I2S_RCR2_BCP |
                (I2S_RCR2_BCD | I2S_RCR2_DIV(1) | I2S_RCR2_MSEL(1));
    I2S1_RCR3 = I2S_RCR3_RCE;
    I2S1_RCR4 = I2S_RCR4_FRSZ(1) | I2S_RCR4_SYWD(31) | I2S_RCR4_MF |
                I2S_RCR4_FSE | I2S_RCR4_FSP | I2S_RCR4_FSD;
    I2S1_RCR5 = I2S_RCR5_WNW(31) | I2S_RCR5_W0W(31) | I2S_RCR5_FBT(31);
    isConfigured = true;
}

void AudioOutputI2S::applyClockConfig(uint32_t sampleRate, bool force)
{
    sampleRateHz = sampleRate;

    int fs = sampleRate;
    int n1 = 4;
    int n2 = 1 + (24000000 * 27) / (fs * 256 * n1);

    double c = ((double)fs * 256 * n1 * n2) / 24000000;
    int c0 = c;
    int c2 = 10000;
    int c1 = c * c2 - (c0 * c2);
    setAudioClock(c0, c1, c2, force);

    CCM_CSCMR1 = (CCM_CSCMR1 & ~(CCM_CSCMR1_SAI1_CLK_SEL_MASK)) |
                 CCM_CSCMR1_SAI1_CLK_SEL(2);
    CCM_CS1CDR = (CCM_CS1CDR & ~(CCM_CS1CDR_SAI1_CLK_PRED_MASK | CCM_CS1CDR_SAI1_CLK_PODF_MASK)) |
                 CCM_CS1CDR_SAI1_CLK_PRED(n1 - 1) |
                 CCM_CS1CDR_SAI1_CLK_PODF(n2 - 1);

    IOMUXC_GPR_GPR1 = (IOMUXC_GPR_GPR1 & ~(IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL_MASK)) |
                      (IOMUXC_GPR_GPR1_SAI1_MCLK_DIR | IOMUXC_GPR_GPR1_SAI1_MCLK1_SEL(0));
}
