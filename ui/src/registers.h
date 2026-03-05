#include <Arduino.h>
#include "../../common/filter.h"
#include "../../common/constants.h"

class RegisterBank {
    public:
        void updateRegister(uint8_t reg, uint32_t value);

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
        };

        RegisterState buffers[2] = {};
        volatile uint8_t readBufferIndex = 0;
        volatile uint8_t writeBufferIndex = 1;
        volatile bool ready = false;
};