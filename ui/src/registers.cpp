#include "registers.h"
#include "netconv.h"

void RegisterBank::updateRegister(uint8_t reg, uint32_t value) {
    RegisterState& w = buffers[writeBufferIndex];

    switch (reg) {
        case REG_OUT_GAIN:
            w.outputGain = ntohf(value);
            break;
        case REG_IN_GAIN:
            w.inputGain = ntohf(value);
            break;
        case REG_FILTER_SELECT:
            w.filterSelect = max(uint8_t(0), min(uint8_t(ntohl(value)), uint8_t(FILTER_BANDS - 1)));
            break;
        case REG_FILTER_FREQ:
            w.frequency = ntohf(value);
            break;
        case REG_FILTER_Q:
            w.q = ntohf(value);
            break;
        case REG_FILTER_GAIN:
            w.filterGain = ntohf(value);
            break;
        case REG_SAMPLE_RATE:
            w.sampleRate = ntohl(value);
            break;
        case REG_RESET:
            w.reset = ntohl(value);
            break;
        case REG_FILTER_TYPE:
            w.filterType = ntohl(value);
            break;
        case REG_FILTER_COEFF0:
        case REG_FILTER_COEFF1:
        case REG_FILTER_COEFF2:
        case REG_FILTER_COEFF3:
        case REG_FILTER_COEFF4:
            w.filterCoeff[w.filterSelect][reg - REG_FILTER_COEFF0] = ntohf(value);
            break;
        case REG_DISPLAY_MODE:
            w.displayMode = ntohl(value);
            break;
        case REG_COMMIT:
            noInterrupts();
            {
                const uint8_t oldRead = readBufferIndex;
                readBufferIndex = writeBufferIndex;
                writeBufferIndex = oldRead;

                // Keep next write buffer in sync so partial updates work as expected
                buffers[writeBufferIndex] = buffers[readBufferIndex];
                ready = true;
            }
            interrupts();
            break;
        default:
            // Handle invalid register
            break;
    }
}