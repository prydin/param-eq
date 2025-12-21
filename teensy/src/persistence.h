#include <Arduino.h>
#include "filter.h"

// A a struct of all user settings that need to be persisted
struct PersistedSettings
{
    uint32_t magic;                            // Magic number for validation
    uint32_t version;                          // Version number for future compatibility
    FilterSettings filterSettings[FILTER_BANDS]; // Settings for low, mid, high bands
    uint8_t selectedFilterBand;                // Currently selected filter band
    uint32_t checksum;                         // Checksum for data integrity
};

bool saveSettings(const PersistedSettings settings);
bool loadSettings(PersistedSettings& settings);
void clearSettings();   