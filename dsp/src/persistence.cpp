// Copyright (c) 2026 Pontus Rydin
// SPDX-License-Identifier: MIT
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <EEPROM.h>
#include <ErriezCRC32.h>
#include "persistence.h"

// EEPROM layout constants
#define EEPROM_MAGIC (0x50455145) // 'PEQE' in ASCII
#define EEPROM_VERSION 2
#define EEPROM_START_ADDR 0

namespace
{
struct PersistedSettingsV1
{
    uint32_t magic;
    uint32_t version;
    FilterSettings filterSettings[FILTER_BANDS];
    uint8_t selectedFilterBand;
    uint8_t displayMode;
    float volume;
    uint32_t checksum;
};

uint32_t calculateChecksum(const void *data, size_t sizeWithoutChecksum)
{
    return crc32Buffer(data, sizeWithoutChecksum);
}
}
 
/**
 * @brief Saves settings to EEPROM with integrity checks.
 * 
 * This function writes the provided PersistedSettings structure to EEPROM
 * starting at EEPROM_START_ADDR. Before writing, it calculates and sets
 * the magic number, version, and checksum fields to ensure data integrity.
 * 
 * @param settings The PersistedSettings structure containing user settings to save.
 * @return true if the settings were successfully saved.
 * 
 * @note The checksum is calculated over the entire structure except the checksum field itself.
 */
bool saveSettings(const PersistedSettings settings) {
    PersistedSettings data = {};
    data = settings;
    data.magic = EEPROM_MAGIC;
    data.version = EEPROM_VERSION;
    data.reserved = 0;
    data.checksum = calculateChecksum(&data, sizeof(PersistedSettings) - sizeof(uint32_t));
    //Dump data to serial for debugging
    Serial.printf("Saving settings: magic=0x%08X, version=%d, checksum=0x%08X, filter[0].frequency=%.2f, masterGain=%.2f, volume=%.2f\n", data.magic, data.version, data.checksum, data.filterSettings[0].frequency, data.masterGain, data.volume);
    EEPROM.put(EEPROM_START_ADDR, data);
    return true;
}

/**
 * @brief Loads settings from EEPROM and validates their integrity.
 * 
 * This function reads persisted settings from EEPROM starting at EEPROM_START_ADDR.
 * It performs validation checks including:
 * - Magic number verification to ensure valid EEPROM data
 * - Version number check to ensure compatibility
 * - CRC32 checksum validation to detect data corruption
 * 
 * @param[out] settings Reference to PersistedSettings structure that will be populated
 *                      with the loaded data if validation succeeds. Only modified if
 *                      the function returns true.
 * 
 * @return true if settings were successfully loaded and validated
 * @return false if magic number mismatch, version mismatch, or checksum validation fails
 * 
 * @note If validation fails, the settings parameter remains unchanged.
 * @note The checksum is calculated over the entire structure except the checksum field itself.
 */
bool loadSettings(PersistedSettings& settings) {
    PersistedSettings data = {};
    EEPROM.get(EEPROM_START_ADDR, data);

    if (data.magic != EEPROM_MAGIC) {
        return false;
    }

    if (data.version == EEPROM_VERSION) {
        uint32_t storedChecksum = data.checksum;
        uint32_t calculatedChecksum = calculateChecksum(&data, sizeof(PersistedSettings) - sizeof(uint32_t));
        if (storedChecksum != calculatedChecksum) {
            return false;
        }

        settings = data;
        Serial.printf("Loaded settings: magic=0x%08X, version=%d, checksum=0x%08X, filter[0].frequency=%.2f, masterGain=%.2f, volume=%.2f\n", data.magic, data.version, data.checksum, data.filterSettings[0].frequency, data.masterGain, data.volume);
        return true;
    }

    if (data.version == 1) {
        PersistedSettingsV1 legacyData = {};
        EEPROM.get(EEPROM_START_ADDR, legacyData);
        uint32_t storedChecksum = legacyData.checksum;
        uint32_t calculatedChecksum = calculateChecksum(&legacyData, sizeof(PersistedSettingsV1) - sizeof(uint32_t));
        if (storedChecksum != calculatedChecksum) {
            return false;
        }

        PersistedSettings migrated = {};
        migrated.magic = legacyData.magic;
        migrated.version = EEPROM_VERSION;
        for (int i = 0; i < FILTER_BANDS; ++i) {
            migrated.filterSettings[i] = legacyData.filterSettings[i];
        }
        migrated.selectedFilterBand = legacyData.selectedFilterBand;
        migrated.displayMode = legacyData.displayMode;
        migrated.reserved = 0;
        migrated.masterGain = 0.0f;
        migrated.volume = legacyData.volume;
        migrated.checksum = calculateChecksum(&migrated, sizeof(PersistedSettings) - sizeof(uint32_t));
        settings = migrated;
        Serial.printf("Loaded legacy settings: magic=0x%08X, version=%d, filter[0].frequency=%.2f, masterGain=%.2f, volume=%.2f\n", legacyData.magic, legacyData.version, legacyData.filterSettings[0].frequency, migrated.masterGain, migrated.volume);
        return true;
    }

    return false;
}

/**
 * @brief Clears persisted settings in EEPROM.
 * 
 * This function resets the EEPROM area used for storing PersistedSettings
 * by writing zeros to the entire structure. This effectively clears any
 * previously saved settings.
 * 
 * @note After calling this function, loading settings will fail validation
 *       until new valid settings are saved.
 */
void clearSettings() {
    PersistedSettings data = {0};
    EEPROM.put(EEPROM_START_ADDR, data);
}