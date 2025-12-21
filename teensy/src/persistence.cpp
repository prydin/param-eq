#include <EEPROM.h>
#include <ErriezCRC32.h>
#include "persistence.h"

// EEPROM layout constants
#define EEPROM_MAGIC (0x50455145) // 'PEQE' in ASCII
#define EEPROM_VERSION 2
#define EEPROM_START_ADDR 0

/**
 * @brief Calculates a simple checksum for the persisted settings data.
 * 
 * This function computes a checksum by summing all bytes in the PersistedSettings
 * structure, excluding the checksum field itself. The checksum is used to verify
 * data integrity when loading settings from persistent storage.
 * 
 * @param data The PersistedSettings structure to calculate the checksum for.
 * @return uint32_t The calculated checksum value (sum of all bytes excluding the checksum field).
 * 
 * @note The checksum field is assumed to be the last field in the PersistedSettings structure.
 * @note This is a simple additive checksum and may not detect all forms of data corruption.
 */
uint32_t calculateChecksum(const PersistedSettings& data) {
    uint32_t sum = 0;
    const uint8_t* ptr = (const uint8_t*)&data;
    size_t size = sizeof(PersistedSettings) - sizeof(uint32_t); // Exclude checksum itself
    for (size_t i = 0; i < size; i++) {
        sum += ptr[i];
    }
    return sum;
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
    PersistedSettings data = settings;
    data.magic = EEPROM_MAGIC;
    data.version = EEPROM_VERSION;
    data.checksum = crc32Buffer(&data, sizeof(PersistedSettings) - sizeof(uint32_t));
    //Dump data to serial for debugging
    Serial.printf("Saving settings: magic=0x%08X, version=%d, checksum=0x%08X, filter[0].frequency=%.2f\n", data.magic, data.version, data.checksum, data.filterSettings[0].frequency);
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
    PersistedSettings data;
    EEPROM.get(EEPROM_START_ADDR, data);
    
    // Validate magic number and version
    if (data.magic != EEPROM_MAGIC || data.version != EEPROM_VERSION) {
        return false;
    }
    
    // Validate checksum
    uint32_t stored_checksum = data.checksum;
    uint32_t calculated_checksum = crc32Buffer(&data, sizeof(PersistedSettings) - sizeof(uint32_t));
    if (stored_checksum != calculated_checksum) {
        return false;
    }
    
    settings = data;
    Serial.printf("Loaded settings: magic=0x%08X, version=%d, checksum=0x%08X, filter[0].frequency=%.2f\n", data.magic, data.version, data.checksum, data.filterSettings[0].frequency);
    return true;
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