#include <arm_math.h>
#include "audio_component.h"
#include "../../common/constants.h"

// FFT window size. Must be a power of 2 and a multiple of AUDIO_BLOCK_SAMPLES.
// Increase for finer frequency resolution (e.g. 512, 1024).
static constexpr int FFT_SIZE = 256;
static constexpr int FFT_SPLIT_DECIMATION = 4;
static constexpr float FFT_SPLIT_CROSSOVER_HZ = 500.0f;

// Display amplitude scaling (linear magnitude, default path).
// Increase FFT_LINEAR_GAIN if the bars are too short.
static constexpr float FFT_LINEAR_GAIN = 4.0f;
static constexpr float FFT_DB_FLOOR = -80.0f;
static constexpr float FFT_DB_CEILING = -12.0f;
static constexpr float FFT_MAG_NOISE_FLOOR = 0.003f;

// Uncomment to enable power-spectrum path (|X[k]|^2) instead of magnitude.
// #define FFT_USE_POWER_SPECTRUM

// Aggregation mode between display updates.
// Default (no define): peak-hold per bin (better transient visibility).
// #define FFT_AGGREGATE_AVERAGE

class AudioFFT32 : public AudioComponent
{
public:
    enum class AmplitudeScale : uint8_t
    {
        Linear,
        Decibels,
    };

    AudioFFT32();

    void process(AudioBuffer *block) override;

    // Returns aggregated FFT bins since the previous call, then resets the aggregate.
    // Returns false when no new FFT frame has been aggregated yet.
    bool doFFT(sample_t *left, sample_t *right);

    // Call whenever the sample rate changes so the display bin map is recomputed.
    void setSampleRate(uint32_t rate);
    void setAmplitudeScale(AmplitudeScale scale);
    AmplitudeScale getAmplitudeScale() const { return amplitudeScale; }
    void setEnabled(bool enabled) { isEnabled = enabled; }
    bool getEnabled() const { return isEnabled; }

private:
    static constexpr int FFT_BINS  = FFT_SIZE / 2;
    static constexpr float LOG_F_MIN = 100.0f;

    static_assert(FFT_SIZE >= AUDIO_BLOCK_SAMPLES,
                  "FFT_SIZE must be >= AUDIO_BLOCK_SAMPLES");
    static_assert((FFT_SIZE % AUDIO_BLOCK_SAMPLES) == 0,
                  "FFT_SIZE must be a multiple of AUDIO_BLOCK_SAMPLES");
    static_assert(FFT_SPLIT_DECIMATION >= 1,
                  "FFT_SPLIT_DECIMATION must be >= 1");

    // Sample accumulation for the next FFT frame.
    sample_t accumLeft[FFT_SIZE];
    sample_t accumRight[FFT_SIZE];
    volatile int writePos = 0;

    // Decimated (low-band) sample accumulation for split FFT.
    sample_t lowAccumLeft[FFT_SIZE];
    sample_t lowAccumRight[FFT_SIZE];
    volatile int lowWritePos = 0;
    int decimPhase = 0;
    sample_t decimAccLeft = 0.0f;
    sample_t decimAccRight = 0.0f;
    bool lowSpectrumReady = false;

    // FFT work buffers reused per frame (avoids large stack usage in ISR).
    sample_t workLeft[FFT_SIZE];
    sample_t workRight[FFT_SIZE];
    sample_t fftOutLeft[FFT_SIZE];
    sample_t fftOutRight[FFT_SIZE];
    sample_t magLeft[FFT_BINS];
    sample_t magRight[FFT_BINS];
    sample_t lowMagLeft[FFT_BINS] = {};
    sample_t lowMagRight[FFT_BINS] = {};

    // Per-frame display bins and aggregated bins since last doFFT().
    sample_t frameBinsLeft[FFT_DISPLAY_BINS];
    sample_t frameBinsRight[FFT_DISPLAY_BINS];
    sample_t aggregatedBinsLeft[FFT_DISPLAY_BINS] = {};
    sample_t aggregatedBinsRight[FFT_DISPLAY_BINS] = {};
    volatile uint32_t aggregatedFrames = 0;

    arm_rfft_fast_instance_f32 fftInstance;

    uint32_t sampleRate = 48000;
    volatile AmplitudeScale amplitudeScale = AmplitudeScale::Linear;
    bool isEnabled = true;

    // Precomputed mapping from display band index → [binStart, binEnd] FFT bin range.
    int binStartHigh[FFT_DISPLAY_BINS];
    int binEndHigh[FFT_DISPLAY_BINS];
    int binStartLow[FFT_DISPLAY_BINS];
    int binEndLow[FFT_DISPLAY_BINS];
    float binCenterHz[FFT_DISPLAY_BINS];

    void computeSpectrum(sample_t *leftInput, sample_t *rightInput,
                         sample_t *leftMagOut, sample_t *rightMagOut);
    void computeFrameBins(const sample_t *inputLeft, const sample_t *inputRight);
    void aggregateFrameBins();
    void computeLogBinMap();
};

