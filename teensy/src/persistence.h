#include <Arduino.h>
#include "filter.h"

// A a struct of all user settings that need to be persisted
struct PersistedSettings
{
    uint32_t magic;                            // Magic number for validation
    uint32_t version;                          // Version number for future compatibility
    float filterFrequencies[FILTER_BANDS];     // Frequencies for low, mid, high bands
    float filterQs[FILTER_BANDS];              // Q factors for low, mid, high bands
    float filterGains[FILTER_BANDS];           // Gains for low, mid, high bands
    uint8_t selectedFilterBand;                // Currently selected filter band
    uint8_t filterTypes[FILTER_BANDS]; // Currently selected filter type
    uint32_t checksum;                         // Checksum for data integrity
};

bool saveSettings(const PersistedSettings settings);
bool loadSettings(PersistedSettings& settings);
void clearSettings();   