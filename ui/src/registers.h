#include <Arduino.h>
#include <stddef.h>
#include "../../common/filter.h"
#include "../../common/constants.h"

class RegisterBank {
    public:
        void updateRegister(uint8_t reg, uint32_t value);

        bool takeVuMeterUpdate(uint16_t &leftLevel, uint16_t &rightLevel) {
            noInterrupts();
            if (!vuMeterDirty) {
                interrupts();
                return false;
            }

            const uint32_t packedLevel = vuMeterPacked;
            vuMeterDirty = false;
            interrupts();

            leftLevel = static_cast<uint16_t>(packedLevel >> 16);
            rightLevel = static_cast<uint16_t>(packedLevel & 0xFFFFU);
            return true;
        }

        bool isReady() {
            noInterrupts();
            if (!ready) {
                interrupts();
                return false;
            }
            ready = false;
            interrupts();
            return true;
        }

        // Getters for all registers (read buffer only)
        float getOutputGain() { return buffers[readBufferIndex].outputGain; }
        float getInputGain() { return buffers[readBufferIndex].inputGain; }
        uint8_t getFilterSelect() { return buffers[readBufferIndex].filterSelect; }
        float getFrequency() { return buffers[readBufferIndex].frequency; }
        float getQ() { return buffers[readBufferIndex].q; }
        float getFilterGain() { return buffers[readBufferIndex].filterGain; }
        uint32_t getSampleRate() { return buffers[readBufferIndex].sampleRate; }
        uint32_t getBypass() { return buffers[readBufferIndex].bypass; }
        uint32_t getReset() { return buffers[readBufferIndex].reset; }
        uint32_t getFilterType() { return buffers[readBufferIndex].filterType; }
        float getFilterCoeff(uint8_t band, uint8_t index) { return buffers[readBufferIndex].filterCoeff[band][index]; }
        uint32_t getDisplayMode() { return buffers[readBufferIndex].displayMode; }
        uint32_t getUiMode() { return buffers[readBufferIndex].uiMode; }
        uint32_t getVuMeterPacked() {
            noInterrupts();
            const uint32_t packedLevel = vuMeterPacked;
            interrupts();
            return packedLevel;
        }
        void copyFftBins(uint8_t *leftBins, uint8_t *rightBins, size_t binCount) {
            const size_t count = (binCount > 16U) ? 16U : binCount;
            const RegisterState &r = buffers[readBufferIndex];

            for (size_t i = 0; i < count; i++) {
                leftBins[i] = r.fftLeft[i];
                rightBins[i] = r.fftRight[i];
            }
        }

    private:
        struct RegisterState {
            float outputGain = 0.0f;
            float inputGain = 0.0f;
            uint32_t sampleRate = 0;
            uint8_t filterSelect = 0;
            float frequency = 0.0f;
            float q = 0.0f;
            float filterGain = 0.0f;
            uint32_t bypass = 0;
            uint32_t reset = 0;
            uint32_t filterType = 0;
            float filterCoeff[FILTER_BANDS][5] = {};
            uint32_t displayMode = 0;
            uint32_t uiMode = 1;
            uint8_t fftLeft[16] = {};
            uint8_t fftRight[16] = {};
            uint32_t fftLeftWords[5] = {};
            uint32_t fftRightWords[4] = {};
            bool fftLeftUsesReg10 = false;
        };

        RegisterState buffers[2] = {};
        volatile uint8_t readBufferIndex = 0;
        volatile uint8_t writeBufferIndex = 1;
        volatile bool ready = false;
        volatile uint32_t vuMeterPacked = 0;
        volatile bool vuMeterDirty = false;
};