/**
 * @file ES9039Q2M.h
 * @brief Driver library for the ES9039Q2M DAC (Digital-to-Analog Converter)
 * 
 * This library provides a comprehensive interface for controlling the ES9039Q2M
 * high-performance stereo DAC via I2C communication. It includes support for:
 * 
 * - System configuration and control (reset, clock management, operating modes)
 * - Audio input selection (PCM, DSD, DOP, S/PDIF)
 * - Volume control and muting for dual channels
 * - Digital filter configuration (IIR, FIR bypass options)
 * - TDM (Time Division Multiplexing) configuration
 * - GPIO control and PWM output
 * - THD (Total Harmonic Distortion) compensation
 * - Automute functionality with configurable thresholds
 * - Master/slave clock modes and PLL bandwidth control
 * 
 * The ES9039Q2M supports various audio formats and sample rates, with advanced
 * features like automatic format detection, programmable filters, and extensive
 * signal path customization.
 * 
 * @author Pontus Rydin
 * @version 1.0
 * @date 2026
 * 
 * @note This driver requires the Arduino Wire library for I2C communication
 * @note Default I2C address is 0x48
 * 
 * @example
 * ES9039Q2M dac;
 * dac.begin();
 * dac.enableDAC();
 * dac.setCH1Volume(200);
 * dac.selectPCMInput();
 */
#ifndef ES9039Q2M_H
#define ES9039Q2M_H

#include <Arduino.h>
#include <Wire.h>

// ES9039Q2M DAC Register Definitions
// I2C Address
#define ES9039Q2M_I2C_ADDR 0x48

// System Registers
#define ES9039Q2M_REG_SYSTEM_CONFIG 0
#define ES9039Q2M_REG_MODE_CONFIG 1
#define ES9039Q2M_REG_DAC_CLOCK_CONFIG 3
#define ES9039Q2M_REG_CLOCK_CONFIG 4
#define ES9039Q2M_REG_CLK_GEAR 5
#define ES9039Q2M_REG_INTERRUPT_MASKP 10
#define ES9039Q2M_REG_INTERRUPT_MASKN 16
#define ES9039Q2M_REG_INTERRUPT_CLEAR 20
#define ES9039Q2M_REG_PLL_BW 29
#define ES9039Q2M_REG_DATA_PATH_CONFIG 34
#define ES9039Q2M_REG_PCM_4X_GAIN 35
#define ES9039Q2M_REG_GPIO12_CONFIG 37
#define ES9039Q2M_REG_GPIO34_CONFIG 38
#define ES9039Q2M_REG_GPIO56_CONFIG 39
#define ES9039Q2M_REG_GPIO78_CONFIG 40
#define ES9039Q2M_REG_GPIO_OUTPUT_ENABLE 41
#define ES9039Q2M_REG_GPIO_INPUT_ENABLE 42
#define ES9039Q2M_REG_GPIO_WK_ENABLE 43
#define ES9039Q2M_REG_GPIO_INVERT 44
#define ES9039Q2M_REG_GPIO_READ 45
#define ES9039Q2M_REG_GPIO_OUTPUT_LOGIC 46
#define ES9039Q2M_REG_PWM1_COUNT 48
#define ES9039Q2M_REG_PWM1_FREQ 49
#define ES9039Q2M_REG_PWM2_COUNT 51
#define ES9039Q2M_REG_PWM2_FREQ 52
#define ES9039Q2M_REG_PWM3_COUNT 54
#define ES9039Q2M_REG_PWM3_FREQ 55
#define ES9039Q2M_REG_INPUT_SELECTION 57
#define ES9039Q2M_REG_MASTER_ENCODER_CONFIG 58
#define ES9039Q2M_REG_TDM_CONFIG 59
#define ES9039Q2M_REG_TDM_CONFIG1 60
#define ES9039Q2M_REG_TDM_CONFIG2 61
#define ES9039Q2M_REG_BCK_WS_MONITOR_CONFIG 62
#define ES9039Q2M_REG_BCK_CH1_SLOT_CONFIG 64
#define ES9039Q2M_REG_BCK_CH2_SLOT_CONFIG 65
#define ES9039Q2M_REG_VOLUME_CH1 74
#define ES9039Q2M_REG_VOLUME_CH2 75
#define ES9039Q2M_REG_VOL_RATE_UP 82
#define ES9039Q2M_REG_VOL_RATE_DOWN 83
#define ES9039Q2M_REG_VOL_RATE_FAST 84
#define ES9039Q2M_REG_DAC_MUTE 86
#define ES9039Q2M_REG_DAC_INVERT 87
#define ES9039Q2M_REG_FILTER_SHAPE 88
#define ES9039Q2M_REG_IIR_BW_SPDIF_SEL 89
#define ES9039Q2M_REG_DAC_PATH_CONFIG 90
#define ES9039Q2M_REG_THD_C2_CH1 91
#define ES9039Q2M_REG_THD_C2_CH2 93  
#define ES9039Q2M_REG_THD_C3_CH1 107
#define ES9039Q2M_REG_THD_C3_CH2 109 
#define ES9039Q2M_AUTOMUTE_ENABLE 123
#define ES9039Q2M_AUTOMUTE_TIME 124
#define ES9039Q2M_AUTOMUTE_LEVEL 126
#define ES9039Q2M_AUTOMUTE_OFF_LEVEL 128
#define ES9039Q2M_AUTOMUTE_SOFT_RAMP 130
#define ES9039Q2M_PROGRAM_RAM_CONTROL 135
#define ES9039Q2M_SPDIF_READ_CONTROL 136
#define ES9039Q2M_PROGRAM_RAM_ADDRESS 137
#define ES9039Q2M_PROGRAM_RAM_DATA 138

// Read only registers
#define ES9039Q2M_CHIP_ID 225
#define ES9039Q2M_INTERRUPT_STATES 229
#define ES9039Q2M_INTERRUPT_SOURCE 234
#define ES9039Q2M_GPIO_READBACK 240
#define ES9039Q2M_GPIO_VOL_MIN_READ 241
#define ES9039Q2M_GPIO_AUTOMUTE_READ 241

// SYSTEM_CONFIG bits
#define ES9039Q2M_BIT_RESET 0x80
#define ES9039Q2M_BIT_64FS_MODE 0x40
#define ES9039Q2M_DAC_ENABLED 0x02

// SYSTEM_MODE_CONFIG bits
#define ES9039Q2M_BIT_DAC_CLK 0x80
#define ES9039Q2M_BIT_SYNC_MODE 0x40
#define ES9039Q2M_BIT_SPDIF_DECODE 0x08
#define ES9039Q2M_BIT_DOP_DECODE 0x04
#define ES9039Q2M_BIT_DSD_DECODE 0x02
#define ES9039Q2M_BIT_TDM_DECODE 0x01

// DAC CLOCK MODE BITS
#define ES9039Q2M_BIT_AUTO_FS_DETECT 0x80
#define ES9039Q2M_BIT_SELECT_IDAC_HALF 0x40
#define ES9039Q2M_BIT_SELECT_IDAC_NUM 0x3f

// Clock gear bits
#define ES9039Q2M_BIT_CLK_GEAR 0x30
#define ES9039Q2M_BIT_AUTO_CLK_GEAR 0x04
#define ES9039Q2M_BIT_AUTO_FS_DETECT_BLOCK_64FS 0x01

// PLL Bandwidth bits
#define ES9039Q2M_BIT_PLL_BW_LOWEST 0x01
#define ES9039Q2M_BIT_PLL_BW_HIGHEST 0x0f

// Data Path Config bits
#define ES9039Q2M_BIT_DATA_PATH_MONO 0x80
#define ES9039Q2M_BIT_DATA_PATH_CALIBRATION 0x40

// PCM 4X Gain bits
#define ES9039Q2M_BIT_CH1_PCM_4X_GAIN 0x01
#define ES9039Q2M_BIT_CH2_PCM_4X_GAIN 0x02

// Input selection bits
#define ES9039Q2M_BIT_AUTO_CH_DETECT 0x80
#define ES9039Q2M_BIT_ENABLE_DSD_FAULT_DETECTION 0x40
#define ES9039Q2M_BIT_DSD_MASTER_MODE 0x20
#define ES9039Q2M_BIT_PCM_MASTER_MODE 0x10
#define ES9039Q2M_BIT_INPUT_PCM 0x00
#define ES9039Q2M_BIT_INPUT_DSD 0x01
#define ES9039Q2M_BIT_INPUT_DOP 0x02
#define ES9039Q2M_BIT_INPUT_SPDIF 0x03
#define ES9039Q2M_BIT_AUTO_INPUT_SELECT 0x01

// Encoder config bits
#define ES9039Q2M_BIT_TDM_RESYNC 0x80
#define ES9039Q2M_BIT_BCK_INV 0x40
#define ES9039Q2M_BIT_MASTER_FRAME_LENGTH_MASK (1 << 4 |  1<<3)
#define ES9039Q2M_BIT_MASTER_FRAME_LENGTH_SHIFT 3
#define ES9039Q2M_BIT_MASTER_WS_PULSE 0x04
#define ES9039Q2M_BIT_MASTER_WS_INV 0x02
#define ES9039Q2M_BIT_MASTER_BCK_INV 0x01

// TDM config bits first set
#define ES9039Q2M_BIT_TDM_LJ_MODE 0x80
#define ES9039Q2M_BIT_TDM_VALID_EDGE 0x40
#define ES9039Q2M_BIT_TDM_DAISY_CHAIN 0x20

// TDM config bits second set
#define ES9039Q2M_BIT_TDM_BIT_WIDTH_MASK (1 << 6 | 1 << 5)
#define ES9039Q2M_BIT_TDM_BIT_WIDTH_SHIFT 5
#define ES9039Q2M_BIT_TDM_LATCH_ADJUST_MASK (1 << 4 | 1 << 3 | 1 << 2 | 1 << 1 | 1)
#define ES9039Q2M_BIT_TDM_32_BITS 0x00
#define ES9039Q2M_BIT_TDM_24_BITS 0x01
#define ES9039Q2M_BIT_TDM_16_BITS 0x02

// BCK/WS Monitor Config bits
#define ES9039Q2M_BIT_DISABLE_DSD_DC 0x80
#define ES9039Q2M_BIT_DISABLE_DSD_MUTE 0x40
#define ES9039Q2M_BIT_ENABLE_WS_MONITOR 0x20
#define ES9039Q2M_BIT_ENABLE_BCK_MONITOR 0x10
#define ES9039Q2M_BIT_DISABLE_PCM_DC 0x08

// Slot config bits
#define ES9039Q2M_BIT_DSD_SLOT_SOURCE_MASK (1 << 7 | 1 << 6 | 1 << 5)
#define ES9039Q2M_BIT_DSD_SLOT_SOURCE_SHIFT 5
#define ES9039Q2M_BIT_TDM_SLOT_SEL_MASK (1 << 4 | 1 << 3 | 1 << 2 | 1 << 1 | 1)

// DAC mute bits
#define ES9039Q2M_BIT_DAC_MUTE_CH2 0x02
#define ES9039Q2M_BIT_DAC_MUTE_CH1 0x01

// DAC invert bits
#define ES9039Q2M_BIT_DAC_INVERT_CH2 0x02
#define ES9039Q2M_BIT_DAC_INVERT_CH1 0x01

// Filter shape constants
#define ES9039Q2M_BIT_FILTER_SHAPE_MASK (1 << 2 | 1 << 1 | 1)
#define ES9039Q2M_FILTER_SHAPE_MIN_PH 0x00
#define ES9039Q2M_FILTER_SHAPE_LIN_PH_FAST_RO_APO 0x01
#define ES9039Q2M_FILTER_SHAPE_LIN_PH_FAST_RO 0x02
#define ES9039Q2M_FILTER_SHAPE_LIN_PH_FAST_RO_LOW_RIP 0x03
#define ES9039Q2M_FILTER_SHAPE_LIN_PH_SLOW_RO 0x04
#define ES9039Q2M_FILTER_SHAPE_MIN_PH_FAST_RO 0x05
#define ES9039Q2M_FILTER_SHAPE_MIN_PH_SLOW_RO 0x06
#define ES9039Q2M_FILTER_SHAPE_MIN_PH_SLOW_RO_DISP 0x07

// IIR Bandwidth and SPDIF Selection constants
#define ES9039Q2M_BIT_IIR_BW_MASK (1 << 2 | 1 << 1 | 1)
#define ES9039Q2M_BIT_SPDIF_SEL_MASK (1 << 7 | 1 << 6 | 1 << 5 | 1 << 4)
#define ES9039Q2M_BIT_SPDIF_SEL_SHIFT 4
#define ES9039Q2M_BIT_VOLUME_HOLD 0x08
#define ES9039Q2M_BIT_BW_X_8 0x01
#define ES9039Q2M_BIT_BW_X_4 0x02
#define ES9039Q2M_BIT_BW_X_2 0x03
#define ES9039Q2M_BIT_BW 0x04
#define ES9039Q2M_BIT_BW_DIV_2 0x05
#define ES9039Q2M_BIT_BW_DIV_4 0x06
#define ES9039Q2M_BIT_BW_DIV_8 0x07

// DAC Path Config Register
#define ES9039Q2M_BIT_BYPASS_IIR 0x04
#define ES9039Q2M_BIT_BYPASS_FIR_4X 0x02
#define ES9039Q2M_BIT_BYPASS_FIR_2X 0x01

// Automute enable bits
#define ES9039Q2M_BIT_EN_CH2 0x02
#define ES9039Q2M_BIT_EN_CH1 0x01

// Automute time bits
#define ES9039Q2M_BIT_EXTERNAL_DEM_IE 0x8000
#define ES9039Q2M_BIT_EXTERNAL_DEM 0x4000
#define ES9039Q2M_BIT_MAP_TO_GROUND 0x0800
#define ES9039Q2M_BIT_AUTOMUTE_TIME_MASK 0x07FF

// Automute soft ramp bits
#define ES9039Q2M_BIT_AUTOMUTE_SOFT_RAMP_MASK (1 << 4 | 1 << 3 | 1 << 2 | 1 << 1 | 1)

// Program RAM control bits
#define ES9039Q2M_BIT_SPDIF_LOAD_USER_BITS 0x80
#define ES9039Q2M_BIT_PROGRAM_COEFF_WE 0x02
#define ES9039Q2M_BIT_PROGRAM_COEFF_EN 0x02

// SPDIF read control bits
#define ES9039Q2M_BIT_SPDIF_READ_CONTROL_MASK (1 << 4 | 1 << 3 | 1 << 2 | 1 << 1 | 1)

// Interrupt status bits
#define ES9039Q2M_BIT_INPUT_SELECT_OVERRIDE_STATE_MASK (1 << 15 | 1 << 14)
#define ES9039Q2M_BIT_INPUT_SELECT_OVERRIDE_STATE_SHIFT 14
#define ES9039Q2M_BIT_TDM_DATA_VALID_STATE 0x4000
#define ES9039Q2M_BIT_WS_FAIL_STATE 0x0080
#define ES9039Q2M_BIT_DOP_VALID_STATE 0x0040
#define ES9039Q2M_BIT_SS_FULL_RAMP_STATE_MASK (1 << 5 | 1 << 4 )
#define ES9039Q2M_BIT_SS_FULL_RAMP_STATE_SHIFT 4
#define ES9039Q2M_BIT_SS_AUTOMUTE_STATE_MASK (1 << 3 | 1 << 2 )
#define ES9039Q2M_BIT_SS_AUTOMUTE_STATE_SHIFT 2
#define ES9039Q2M_BIT_SS_VOL_MIN_STATE (1 << 1 | 1)

// Interrupt source bits
#define ES9039Q2M_BIT_INPUT_SELECT_OVERRIDE_SOURCE_MASK (1 << 15 | 1 << 14)
#define ES9039Q2M_BIT_TDM_DATA_VALID_SOURCE 0x4000
#define ES9039Q2M_BIT_WS_FAIL_SOURCE 0x0080
#define ES9039Q2M_BIT_DOP_VALID_SOURCE 0x0040
#define ES9039Q2M_BIT_SS_FULL_RAMP_SOURCE_MASK (1 << 5 | 1 << 4 )
#define ES9039Q2M_BIT_SS_FULL_RAMP_SOURCE_SHIFT 4
#define ES9039Q2M_BIT_SS_AUTOMUTE_SOURCE_MASK (1 << 3 | 1 << 2 )
#define ES9039Q2M_BIT_SS_AUTOMUTE_SOURCE_SHIFT 2
#define ES9039Q2M_BIT_SS_VOL_MIN_SOURCE (1 << 1 | 1)

// Min vol bits
#define ES9039Q2M_BIT_MIN_VOL_CH2 (1 << 1)
#define ES9039Q2M_BIT_MIN_VOL_CH1 (1)

// Automute bits
#define ES9039Q2M_BIT_AUTOMUTE_CH2 (1 << 1)
#define ES9039Q2M_BIT_AUTOMUTE_CH1 (1)


class ES9039Q2M
{
public:
    /**
     * @brief Construct a new ES9039Q2M object
     */
    ES9039Q2M();
    
    /**
     * @brief Destroy the ES9039Q2M object
     */
    ~ES9039Q2M();

    /**
     * @brief Initialize the ES9039Q2M DAC
     * @param address I2C address of the device (default: 0x48)
     * @return true if initialization successful, false otherwise
     */
    bool begin(uint8_t address = ES9039Q2M_I2C_ADDR);
    
    // *********************************************
    // Low level register access
    // *********************************************
    /**
     * @brief Read an 8-bit register
     * @param reg Register address
     * @return uint8_t Register value
     */
    uint8_t readRegister8(uint8_t reg);
    
    /**
     * @brief Write an 8-bit register
     * @param reg Register address
     * @param value Value to write
     */
    void writeRegister8(uint8_t reg, uint8_t value);
    
    /**
     * @brief Write an 8-bit register with bit masking
     * @param reg Register address
     * @param mask Bit mask
     * @param value Value to write
     */
    void writeRegisterMasked8(uint8_t reg, uint8_t mask, uint8_t value);
    
    /**
     * @brief Read a 16-bit register
     * @param reg Register address
     * @return uint16_t Register value
     */
    uint16_t readRegister16(uint8_t reg);
    
    /**
     * @brief Write a 16-bit register
     * @param reg Register address
     * @param value Value to write
     */
    void writeRegister16(uint8_t reg, uint16_t value);
    
    /**
     * @brief Write a 16-bit register with bit masking
     * @param reg Register address
     * @param mask Bit mask
     * @param value Value to write
     */
    void ES9039Q2M::writeRegisterMasked16(uint8_t reg, uint16_t mask, uint16_t value);

    /**
     * @brief Reads a 24-bit signed integer value from a register.
     * 
     * @param reg The register address to read from
     * @return int32_t The 24-bit signed value read from the register, sign-extended to 32 bits
     */
    int32_t readRegister24Signed(uint8_t reg);

    /**
     * @brief Writes a signed 24-bit value to a register.
     * 
     * This function writes a signed 32-bit integer value to a 24-bit register.
     * The value is truncated to fit into 24 bits before writing.
     * 
     * @param reg The register address to write to.
     * @param value The signed 32-bit value to write (will be truncated to 24 bits).
     */
    void writeRegister24Signed(uint8_t reg, int32_t value);

    // *********************************************
    // Medium level functions
    // *********************************************
    // System Config Register Functions
    /**
     * @brief Set the system configuration register
     * @param config Configuration value
     */
    void setSystemConfig(uint8_t config);
    
    /**
     * @brief Get the system configuration register
     * @return uint8_t Current configuration value
     */
    uint8_t getSystemConfig();

    /**
     * @brief Get mode configuration register
     * @return uint8_t Current mode configuration value
     */
    uint8_t getModeConfig() { return readRegister8(ES9039Q2M_REG_MODE_CONFIG); }

    // System config functions
    /**
     * @brief Perform a soft reset of the DAC
     */
    void softReset() { writeRegister8(ES9039Q2M_REG_SYSTEM_CONFIG, readRegister8(ES9039Q2M_REG_SYSTEM_CONFIG) | ES9039Q2M_BIT_RESET); }
    
    /**
     * @brief Enable the DAC
     */
    void enableDAC() { writeRegister8(ES9039Q2M_REG_SYSTEM_CONFIG, readRegister8(ES9039Q2M_REG_SYSTEM_CONFIG) | ES9039Q2M_DAC_ENABLED); }
    
    /**
     * @brief Disable the DAC
     */
    void disableDAC() { writeRegister8(ES9039Q2M_REG_SYSTEM_CONFIG, readRegister8(ES9039Q2M_REG_SYSTEM_CONFIG) & ~ES9039Q2M_DAC_ENABLED); }
    
    /**
     * @brief Check if DAC is enabled
     * @return true if enabled, false otherwise
     */
    bool isDACEnabled() { return (readRegister8(ES9039Q2M_REG_SYSTEM_CONFIG) & ES9039Q2M_DAC_ENABLED) != 0; }
    
    /**
     * @brief Enable 64FS mode
     */
    void enable64FSMode() { writeRegister8(ES9039Q2M_REG_SYSTEM_CONFIG, readRegister8(ES9039Q2M_REG_SYSTEM_CONFIG) | ES9039Q2M_BIT_64FS_MODE); }
    
    /**
     * @brief Disable 64FS mode
     */
    void disable64FSMode() { writeRegister8(ES9039Q2M_REG_SYSTEM_CONFIG, readRegister8(ES9039Q2M_REG_SYSTEM_CONFIG) & ~ES9039Q2M_BIT_64FS_MODE); }
    
    /**
     * @brief Check if 64FS mode is enabled
     * @return true if enabled, false otherwise
     */
    bool is64FSModeEnabled() { return (readRegister8(ES9039Q2M_REG_SYSTEM_CONFIG) & ES9039Q2M_BIT_64FS_MODE) != 0; }

    // System mode config functions
    /**
     * @brief Enable DAC clock
     */
    void enableDACClock() { writeRegister8(ES9039Q2M_REG_MODE_CONFIG, readRegister8(ES9039Q2M_REG_MODE_CONFIG) | ES9039Q2M_BIT_DAC_CLK); }
    
    /**
     * @brief Disable DAC clock
     */
    void disableDACClock() { writeRegister8(ES9039Q2M_REG_MODE_CONFIG, readRegister8(ES9039Q2M_REG_MODE_CONFIG) & ~ES9039Q2M_BIT_DAC_CLK); }
    
    /**
     * @brief Check if DAC clock is enabled
     * @return true if enabled, false otherwise
     */
    bool isDACClockEnabled() { return (readRegister8(ES9039Q2M_REG_MODE_CONFIG) & ES9039Q2M_BIT_DAC_CLK) != 0; }
    
    /**
     * @brief Set synchronous mode
     */
    void syncMode() { writeRegister8(ES9039Q2M_REG_MODE_CONFIG, readRegister8(ES9039Q2M_REG_MODE_CONFIG) | ES9039Q2M_BIT_SYNC_MODE); }
    
    /**
     * @brief Set asynchronous mode
     */
    void asyncMode() { writeRegister8(ES9039Q2M_REG_MODE_CONFIG, readRegister8(ES9039Q2M_REG_MODE_CONFIG) & ~ES9039Q2M_BIT_SYNC_MODE); }
    
    /**
     * @brief Check if synchronous mode is active
     * @return true if sync mode, false if async mode
     */
    bool isSyncMode() { return (readRegister8(ES9039Q2M_REG_MODE_CONFIG) & ES9039Q2M_BIT_SYNC_MODE) != 0; }
    
    /**
     * @brief Enable S/PDIF decoding
     */
    void enableSPDIFDecode() { writeRegister8(ES9039Q2M_REG_MODE_CONFIG, readRegister8(ES9039Q2M_REG_MODE_CONFIG) | ES9039Q2M_BIT_SPDIF_DECODE); }
    
    /**
     * @brief Disable S/PDIF decoding
     */
    void disableSPDIFDecode() { writeRegister8(ES9039Q2M_REG_MODE_CONFIG, readRegister8(ES9039Q2M_REG_MODE_CONFIG) & ~ES9039Q2M_BIT_SPDIF_DECODE); }
    
    /**
     * @brief Check if S/PDIF decoding is enabled
     * @return true if enabled, false otherwise
     */
    bool isSPDIFDecodeEnabled() { return (readRegister8(ES9039Q2M_REG_MODE_CONFIG) & ES9039Q2M_BIT_SPDIF_DECODE) != 0; }
    
    /**
     * @brief Enable DOP (DSD over PCM) decoding
     */
    void enableDOPDecode() { writeRegister8(ES9039Q2M_REG_MODE_CONFIG, readRegister8(ES9039Q2M_REG_MODE_CONFIG) | ES9039Q2M_BIT_DOP_DECODE); }
    
    /**
     * @brief Disable DOP decoding
     */
    void disableDOPDecode() { writeRegister8(ES9039Q2M_REG_MODE_CONFIG, readRegister8(ES9039Q2M_REG_MODE_CONFIG) & ~ES9039Q2M_BIT_DOP_DECODE); }
    
    /**
     * @brief Check if DOP decoding is enabled
     * @return true if enabled, false otherwise
     */
    bool isDOPDecodeEnabled() { return (readRegister8(ES9039Q2M_REG_MODE_CONFIG) & ES9039Q2M_BIT_DOP_DECODE) != 0; }
    
    /**
     * @brief Enable DSD decoding
     */
    void enableDSDDecode() { writeRegister8(ES9039Q2M_REG_MODE_CONFIG, readRegister8(ES9039Q2M_REG_MODE_CONFIG) | ES9039Q2M_BIT_DSD_DECODE); }
    
    /**
     * @brief Disable DSD decoding
     */
    void disableDSDDecode() { writeRegister8(ES9039Q2M_REG_MODE_CONFIG, readRegister8(ES9039Q2M_REG_MODE_CONFIG) & ~ES9039Q2M_BIT_DSD_DECODE); }
    
    /**
     * @brief Check if DSD decoding is enabled
     * @return true if enabled, false otherwise
     */
    bool isDSDDecodeEnabled() { return (readRegister8(ES9039Q2M_REG_MODE_CONFIG) & ES9039Q2M_BIT_DSD_DECODE) != 0; }
    
    /**
     * @brief Enable TDM (Time Division Multiplexing) decoding
     */
    void enableTDMDecode() { writeRegister8(ES9039Q2M_REG_MODE_CONFIG, readRegister8(ES9039Q2M_REG_MODE_CONFIG) | ES9039Q2M_BIT_TDM_DECODE); }
    
    /**
     * @brief Disable TDM decoding
     */
    void disableTDMDecode() { writeRegister8(ES9039Q2M_REG_MODE_CONFIG, readRegister8(ES9039Q2M_REG_MODE_CONFIG) & ~ES9039Q2M_BIT_TDM_DECODE); }
    
    /**
     * @brief Check if TDM decoding is enabled
     * @return true if enabled, false otherwise
     */
    bool isTDMDecodeEnabled() { return (readRegister8(ES9039Q2M_REG_MODE_CONFIG) & ES9039Q2M_BIT_TDM_DECODE) != 0; }

    // DAC Clock mode functions
    /**
     * @brief Enable automatic sample rate detection
     */
    void enableAutoFSDetect() { writeRegister8(ES9039Q2M_REG_DAC_CLOCK_CONFIG, readRegister8(ES9039Q2M_REG_DAC_CLOCK_CONFIG) | ES9039Q2M_BIT_AUTO_FS_DETECT); }
    
    /**
     * @brief Disable automatic sample rate detection
     */
    void disableAutoFSDetect() { writeRegister8(ES9039Q2M_REG_DAC_CLOCK_CONFIG, readRegister8(ES9039Q2M_REG_DAC_CLOCK_CONFIG) & ~ES9039Q2M_BIT_AUTO_FS_DETECT); }
    
    /**
     * @brief Check if automatic sample rate detection is enabled
     * @return true if enabled, false otherwise
     */
      bool isAutoFSDetectEnabled() { return (readRegister8(ES9039Q2M_REG_DAC_CLOCK_CONFIG) & ES9039Q2M_BIT_AUTO_FS_DETECT) != 0; }
    
    /**
     * @brief Set IDAC to half mode
     */
    void setIDACHalf() { writeRegister8(ES9039Q2M_REG_DAC_CLOCK_CONFIG, readRegister8(ES9039Q2M_REG_DAC_CLOCK_CONFIG) | ES9039Q2M_BIT_SELECT_IDAC_HALF); }
    
    /**
     * @brief Get IDAC half mode status
     * @return true if IDAC is in half mode, false otherwise
     */
    bool getIDACHalf() { return (readRegister8(ES9039Q2M_REG_DAC_CLOCK_CONFIG) & ES9039Q2M_BIT_SELECT_IDAC_HALF) != 0; }
    
    /**
     * @brief Set IDAC number
     * @param num IDAC number value (0-63)
     */
    void setIDACNum(uint8_t num) { writeRegister8(ES9039Q2M_REG_DAC_CLOCK_CONFIG, (readRegister8(ES9039Q2M_REG_DAC_CLOCK_CONFIG) & ~ES9039Q2M_BIT_SELECT_IDAC_NUM) | (num & ES9039Q2M_BIT_SELECT_IDAC_NUM)); }
    
    /**
     * @brief Get IDAC number
     * @return uint8_t Current IDAC number value
     */
    uint8_t getIDACNum() { return readRegister8(ES9039Q2M_REG_DAC_CLOCK_CONFIG) & ES9039Q2M_BIT_SELECT_IDAC_NUM; }

    /**
     * @brief Get DAC clock configuration register
     * @return uint8_t Current DAC clock configuration value
     */
    uint8_t getDACClockConfig() { return readRegister8(ES9039Q2M_REG_DAC_CLOCK_CONFIG); }

    // Clock mode functions
    /**
     * @brief Set master BCLK divider
     * @param divider Divider value
     */
    void setMasterBclkDivider(uint8_t divider) { writeRegister8(ES9039Q2M_REG_CLOCK_CONFIG, divider); }
    
    /**
     * @brief Get master BCLK divider
     * @return uint8_t Current divider value
     */
    uint8_t getMasterBclkDivider() { return readRegister8(ES9039Q2M_REG_CLOCK_CONFIG); }

    // Clock gear functions
    /**
     * @brief Set clock gear
     * @param gear Clock gear setting
     */
    void setClockGear(uint8_t gear) { writeRegister8(ES9039Q2M_REG_CLOCK_CONFIG, (readRegister8(ES9039Q2M_REG_CLOCK_CONFIG) & ~(ES9039Q2M_BIT_CLK_GEAR << 4)) | (gear & ES9039Q2M_BIT_CLK_GEAR)); }
    
    /**
     * @brief Enable automatic clock gear adjustment
     */
    void enableAutoClockGear() { writeRegister8(ES9039Q2M_REG_CLOCK_CONFIG, readRegister8(ES9039Q2M_REG_CLOCK_CONFIG) | ES9039Q2M_BIT_AUTO_CLK_GEAR); }
    
    /**
     * @brief Disable automatic clock gear adjustment
     */
    void disableAutoClockGear() { writeRegister8(ES9039Q2M_REG_CLOCK_CONFIG, readRegister8(ES9039Q2M_REG_CLOCK_CONFIG) & ~ES9039Q2M_BIT_AUTO_CLK_GEAR); }
    
    /**
     * @brief Check if automatic clock gear is enabled
     * @return true if enabled, false otherwise
     */
    bool isAutoClockGearEnabled() { return (readRegister8(ES9039Q2M_REG_CLOCK_CONFIG) & ES9039Q2M_BIT_AUTO_CLK_GEAR) != 0; }
    
    /**
     * @brief Get current clock gear setting
     * @return uint8_t Current clock gear value
     */
    uint8_t getClockGear() { return (readRegister8(ES9039Q2M_REG_CLOCK_CONFIG) & ES9039Q2M_BIT_CLK_GEAR) >> 4; }

    /**
     * @brief Get clock configuration register
     * @return uint8_t Current clock configuration value
     */
    uint8_t getClockConfig() { return readRegister8(ES9039Q2M_REG_CLOCK_CONFIG); }

    // System Config Bit Manipulation Functions
    /**
     * @brief Set a specific bit in the system configuration register
     * @param bit Bit position
     * @param value Bit value (true/false)
     */
    void setSystemConfigBit(uint8_t bit, bool value);
    
    /**
     * @brief Get a specific bit from the system configuration register
     * @param bit Bit position
     * @return bool Bit value
     */
    bool getSystemConfigBit(uint8_t bit);

    // Interrupt Mask Functions
    /**
     * @brief Set positive edge interrupt mask
     * @param mask Interrupt mask value
     */
    void setInterruptMaskP(uint16_t mask) { writeRegister16(ES9039Q2M_REG_INTERRUPT_MASKP, mask); }
    
    /**
     * @brief Get positive edge interrupt mask
     * @return uint16_t Current interrupt mask value
     */
    uint16_t getInterruptMasP() { return readRegister16(ES9039Q2M_REG_INTERRUPT_MASKP); }
    
    /**
     * @brief Set negative edge interrupt mask
     * @param mask Interrupt mask value
     */
    void setInterruptMaskN(uint16_t mask) { writeRegister16(ES9039Q2M_REG_INTERRUPT_MASKN, mask); }
    
    /**
     * @brief Get negative edge interrupt mask
     * @return uint16_t Current interrupt mask value
     */
    uint16_t getInterruptMasN() { return readRegister16(ES9039Q2M_REG_INTERRUPT_MASKN); }

    // Interrupt Clear Functions
    /**
     * @brief Clear interrupts
     * @param mask Interrupt clear mask
     */
    void clearInterrupt(uint16_t mask) { writeRegister16(ES9039Q2M_REG_INTERRUPT_CLEAR, mask); }

    // PLL Bandwidth Functions
    /**
     * @brief Set PLL bandwidth
     * @param bw Bandwidth value (0-15)
     */
    void setPLLBandwidth(uint8_t bw) { writeRegister8(ES9039Q2M_REG_PLL_BW, bw << 4); }
    
    /**
     * @brief Get PLL bandwidth
     * @return uint8_t Current bandwidth value
     */
    uint8_t getPLLBandwidth() { return readRegister8(ES9039Q2M_REG_PLL_BW) >> 4; }

    /**
     * @brief Get PLL bandwidth register
     * @return uint8_t Current PLL bandwidth register value
     */
    uint8_t getPLLBWRegister() { return readRegister8(ES9039Q2M_REG_PLL_BW); }

    // Data Path Functions
    /**
     * @brief Enable mono mode
     */
    void enableMono() { writeRegister8(ES9039Q2M_REG_DATA_PATH_CONFIG, readRegister8(ES9039Q2M_REG_DATA_PATH_CONFIG) | ES9039Q2M_BIT_DATA_PATH_MONO); }
    
    /**
     * @brief Disable mono mode
     */
    void disableMono() { writeRegister8(ES9039Q2M_REG_DATA_PATH_CONFIG, readRegister8(ES9039Q2M_REG_DATA_PATH_CONFIG) & ~ES9039Q2M_BIT_DATA_PATH_MONO); }
    
    /**
     * @brief Check if mono mode is enabled
     * @return true if enabled, false otherwise
     */
    bool isMonoEnabled() { return (readRegister8(ES9039Q2M_REG_DATA_PATH_CONFIG) & ES9039Q2M_BIT_DATA_PATH_MONO) != 0; }
    
    /**
     * @brief Enable calibration mode
     */
    void enableCalibration() { writeRegister8(ES9039Q2M_REG_DATA_PATH_CONFIG, readRegister8(ES9039Q2M_REG_DATA_PATH_CONFIG) | ES9039Q2M_BIT_DATA_PATH_CALIBRATION); }
    
    /**
     * @brief Disable calibration mode
     */
    void disableCalibration() { writeRegister8(ES9039Q2M_REG_DATA_PATH_CONFIG, readRegister8(ES9039Q2M_REG_DATA_PATH_CONFIG) & ~ES9039Q2M_BIT_DATA_PATH_CALIBRATION); }
    
    /**
     * @brief Check if calibration mode is enabled
     * @return true if enabled, false otherwise
     */
    bool isCalibrationEnabled() { return (readRegister8(ES9039Q2M_REG_DATA_PATH_CONFIG) & ES9039Q2M_BIT_DATA_PATH_CALIBRATION) != 0; }

    /**
     * @brief Get data path configuration register
     * @return uint8_t Current data path configuration value
     */
    uint8_t getDataPathConfig() { return readRegister8(ES9039Q2M_REG_DATA_PATH_CONFIG); }

    // PCM 4X Gain Functions
    /**
     * @brief Enable 4X gain for channel 1 in PCM mode
     */
    void enableCH1PCM4XGain() { writeRegister8(ES9039Q2M_REG_PCM_4X_GAIN, readRegister8(ES9039Q2M_REG_PCM_4X_GAIN) | ES9039Q2M_BIT_CH1_PCM_4X_GAIN); }
    
    /**
     * @brief Disable 4X gain for channel 1 in PCM mode
     */
    void disableCH1PCM4XGain() { writeRegister8(ES9039Q2M_REG_PCM_4X_GAIN, readRegister8(ES9039Q2M_REG_PCM_4X_GAIN) & ~ES9039Q2M_BIT_CH1_PCM_4X_GAIN); }
    
    /**
     * @brief Enable 4X gain for channel 2 in PCM mode
     */
    void enableCH2PCM4XGain() { writeRegister8(ES9039Q2M_REG_PCM_4X_GAIN, readRegister8(ES9039Q2M_REG_PCM_4X_GAIN) | ES9039Q2M_BIT_CH2_PCM_4X_GAIN); }
    
    /**
     * @brief Disable 4X gain for channel 2 in PCM mode
     */
    void disableCH2PCM4XGain() { writeRegister8(ES9039Q2M_REG_PCM_4X_GAIN, readRegister8(ES9039Q2M_REG_PCM_4X_GAIN) & ~ES9039Q2M_BIT_CH2_PCM_4X_GAIN); }
    
    /**
     * @brief Check if channel 1 PCM 4X gain is enabled
     * @return true if enabled, false otherwise
     */
    bool isCH1PCM4XGainEnabled() { return (readRegister8(ES9039Q2M_REG_PCM_4X_GAIN) & ES9039Q2M_BIT_CH1_PCM_4X_GAIN) != 0; }
    
    /**
     * @brief Check if channel 2 PCM 4X gain is enabled
     * @return true if enabled, false otherwise
     */
    bool isCH2PCM4XGainEnabled() { return (readRegister8(ES9039Q2M_REG_PCM_4X_GAIN) & ES9039Q2M_BIT_CH2_PCM_4X_GAIN) != 0; } 

    /**
     * @brief Get PCM 4X gain register
     * @return uint8_t Current PCM 4X gain value
     */
    uint8_t getPCM4XGain() { return readRegister8(ES9039Q2M_REG_PCM_4X_GAIN); }

    // GPIO Functions
    /**
     * @brief Set GPIO pins 1 and 2 configuration
     * @param config Configuration value
     */
    void setGPIO12Config(uint8_t config) { writeRegister8(ES9039Q2M_REG_GPIO12_CONFIG, config); }
    
    /**
     * @brief Get GPIO pins 1 and 2 configuration
     * @return uint8_t Current configuration value
     */
    uint8_t getGPIO12Config() { return readRegister8(ES9039Q2M_REG_GPIO12_CONFIG); }
    
    /**
     * @brief Set GPIO pins 3 and 4 configuration
     * @param config Configuration value
     */
    void setGPIO34Config(uint8_t config) { writeRegister8(ES9039Q2M_REG_GPIO34_CONFIG, config); }
    
    /**
     * @brief Get GPIO pins 3 and 4 configuration
     * @return uint8_t Current configuration value
     */
    uint8_t getGPIO34Config() { return readRegister8(ES9039Q2M_REG_GPIO34_CONFIG); }
    
    /**
     * @brief Set GPIO pins 5 and 6 configuration
     * @param config Configuration value
     */
    void setGPIO56Config(uint8_t config) { writeRegister8(ES9039Q2M_REG_GPIO56_CONFIG, config); }
    
    /**
     * @brief Get GPIO pins 5 and 6 configuration
     * @return uint8_t Current configuration value
     */
    uint8_t getGPIO56Config() { return readRegister8(ES9039Q2M_REG_GPIO56_CONFIG); }
    
    /**
     * @brief Set GPIO pins 7 and 8 configuration
     * @param config Configuration value
     */
    void setGPIO78Config(uint8_t config) { writeRegister8(ES9039Q2M_REG_GPIO78_CONFIG, config); }
    
    /**
     * @brief Get GPIO pins 7 and 8 configuration
     * @return uint8_t Current configuration value
     */
    uint8_t getGPIO78Config() { return readRegister8(ES9039Q2M_REG_GPIO78_CONFIG); }
    
    /**
     * @brief Set GPIO output enable mask
     * @param enable Enable mask for GPIO outputs
     */
    void setGPIOOutputEnable(uint8_t enable) { writeRegister8(ES9039Q2M_REG_GPIO_OUTPUT_ENABLE, enable); }
    
    /**
     * @brief Get GPIO output enable mask
     * @return uint8_t Current output enable mask
     */
    uint8_t getGPIOOutputEnable() { return readRegister8(ES9039Q2M_REG_GPIO_OUTPUT_ENABLE); }
    
    /**
     * @brief Set GPIO input enable mask
     * @param enable Enable mask for GPIO inputs
     */
    void setGPIOInputEnable(uint8_t enable) { writeRegister8(ES9039Q2M_REG_GPIO_INPUT_ENABLE, enable); }
    
    /**
     * @brief Get GPIO input enable mask
     * @return uint8_t Current input enable mask
     */
    uint8_t getGPIOInputEnable() { return readRegister8(ES9039Q2M_REG_GPIO_INPUT_ENABLE); }
    
    /**
     * @brief Set GPIO wake enable mask
     * @param enable Wake enable mask for GPIOs
     */
    void setGPIOWKEnable(uint8_t enable) { writeRegister8(ES9039Q2M_REG_GPIO_WK_ENABLE, enable); }
    
    /**
     * @brief Get GPIO wake enable mask
     * @return uint8_t Current wake enable mask
     */
    uint8_t getGPIOWKEnable() { return readRegister8(ES9039Q2M_REG_GPIO_WK_ENABLE); }
    
    /**
     * @brief Set GPIO invert mask
     * @param invert Invert mask for GPIOs
     */
    void setGPIOInvert(uint8_t invert) { writeRegister8(ES9039Q2M_REG_GPIO_INVERT, invert); }
    
    /**
     * @brief Get GPIO invert mask
     * @return uint8_t Current invert mask
     */
    uint8_t getGPIOInvert() { return readRegister8(ES9039Q2M_REG_GPIO_INVERT); }
    
    /**
     * @brief Read GPIO pin states
     * @return uint8_t GPIO pin states
     */
    uint8_t readGPIO() { return readRegister8(ES9039Q2M_REG_GPIO_READ); }
    
    /**
     * @brief Set readback value
     * @param value Readback value
     */
    void setReadback(uint8_t value) { writeRegister8(ES9039Q2M_REG_GPIO_READ, value); }
    
    /**
     * @brief Get readback value
     * @return uint8_t Readback value
     */
    uint8_t getReadback() { return readRegister8(ES9039Q2M_REG_GPIO_READ); }
    
    /**
     * @brief Set GPIO output logic levels
     * @param value GPIO output logic value
     */
    void setGPIOOutputLogic(uint16_t value) { writeRegister16(ES9039Q2M_REG_GPIO_OUTPUT_LOGIC, value); }
    
    /**
     * @brief Get GPIO output logic levels
     * @return uint16_t Current GPIO output logic value
     */
    uint16_t getGPIOOutputLogic() { return readRegister16(ES9039Q2M_REG_GPIO_OUTPUT_LOGIC); }

    // PWM Functions
    /**
     * @brief Set PWM1 duty cycle count
     * @param count PWM1 count value
     */
    void setPWM1Count(uint8_t count) { writeRegister8(ES9039Q2M_REG_PWM1_COUNT, count); }
    
    /**
     * @brief Get PWM1 duty cycle count
     * @return uint8_t PWM1 count value
     */
    uint8_t getPWM1Count() { return readRegister8(ES9039Q2M_REG_PWM1_COUNT); }
    
    /**
     * @brief Set PWM1 frequency
     * @param freq PWM1 frequency value
     */
    void setPWM1Freq(uint16_t freq) { writeRegister16(ES9039Q2M_REG_PWM1_FREQ, freq); }
    
    /**
     * @brief Get PWM1 frequency
     * @return uint16_t PWM1 frequency value
     */
    uint16_t getPWM1Freq() { return readRegister16(ES9039Q2M_REG_PWM1_FREQ); }
    
    /**
     * @brief Set PWM2 duty cycle count
     * @param count PWM2 count value
     */
    void setPWM2Count(uint16_t count) { writeRegister16(ES9039Q2M_REG_PWM2_COUNT, count); }
    
    /**
     * @brief Get PWM2 duty cycle count
     * @return uint16_t PWM2 count value
     */
    uint16_t getPWM2Count() { return readRegister16(ES9039Q2M_REG_PWM2_COUNT); }
    
    /**
     * @brief Set PWM2 frequency
     * @param freq PWM2 frequency value
     */
    void setPWM2Freq(uint16_t freq) { writeRegister16(ES9039Q2M_REG_PWM2_FREQ, freq); }
    
    /**
     * @brief Get PWM2 frequency
     * @return uint16_t PWM2 frequency value
     */
    uint16_t getPWM2Freq() { return readRegister16(ES9039Q2M_REG_PWM2_FREQ); }
    
    /**
     * @brief Set PWM3 duty cycle count
     * @param count PWM3 count value
     */
    void setPWM3Count(uint8_t count) { writeRegister8(ES9039Q2M_REG_PWM3_COUNT, count); }
    
    /**
     * @brief Get PWM3 duty cycle count
     * @return uint8_t PWM3 count value
     */
    uint8_t getPWM3Count() { return readRegister8(ES9039Q2M_REG_PWM3_COUNT); }
    
    /**
     * @brief Set PWM3 frequency
     * @param freq PWM3 frequency value
     */
    void setPWM3Freq(uint16_t freq) { writeRegister16(ES9039Q2M_REG_PWM3_FREQ, freq); }
    
    /**
     * @brief Get PWM3 frequency
     * @return uint16_t PWM3 frequency value
     */
    uint16_t getPWM3Freq() { return readRegister16(ES9039Q2M_REG_PWM3_FREQ); }

    // Input selection functions
    /**
     * @brief Set input selection register
     * @param selection Input selection value
     */
    void setInputSelection(uint8_t selection) { writeRegister8(ES9039Q2M_REG_INPUT_SELECTION, selection); }
    
    /**
     * @brief Get input selection register
     * @return uint8_t Current input selection value
     */
    uint8_t getInputSelection() { return readRegister8(ES9039Q2M_REG_INPUT_SELECTION); }
    
    /**
     * @brief Select PCM input
     */
    void selectPCMInput() { writeRegister8(ES9039Q2M_REG_INPUT_SELECTION, ES9039Q2M_BIT_INPUT_PCM << 1); }
    
    /**
     * @brief Select DSD input
     */
    void selectDSDInput() { writeRegister8(ES9039Q2M_REG_INPUT_SELECTION, ES9039Q2M_BIT_INPUT_DSD << 1); }
    
    /**
     * @brief Select DOP input
     */
    void selectDOPInput() { writeRegister8(ES9039Q2M_REG_INPUT_SELECTION, ES9039Q2M_BIT_INPUT_DOP << 1); }
    
    /**
     * @brief Select S/PDIF input
     */
    void selectSPDIFInput() { writeRegister8(ES9039Q2M_REG_INPUT_SELECTION, ES9039Q2M_BIT_INPUT_SPDIF << 1); }
    
    /**
     * @brief Enable automatic input selection
     */
    void enableAutoInputSelect() { writeRegisterMasked8(ES9039Q2M_REG_INPUT_SELECTION, ES9039Q2M_BIT_AUTO_INPUT_SELECT, ES9039Q2M_BIT_AUTO_INPUT_SELECT); }
    
    /**
     * @brief Disable automatic input selection
     */
    void disableAutoInputSelect() { writeRegisterMasked8(ES9039Q2M_REG_INPUT_SELECTION, ES9039Q2M_BIT_AUTO_INPUT_SELECT, 0); }
    
    /**
     * @brief Check if automatic input selection is enabled
     * @return true if enabled, false otherwise
     */
    bool isAutoInputSelectEnabled() { return (readRegister8(ES9039Q2M_REG_INPUT_SELECTION) & ES9039Q2M_BIT_AUTO_INPUT_SELECT) != 0; }

    /**
     * @brief Enables PCM master mode 
     */
    void enablePCMMasterMode() { writeRegister8(ES9039Q2M_REG_INPUT_SELECTION, readRegister8(ES9039Q2M_REG_INPUT_SELECTION) | ES9039Q2M_BIT_PCM_MASTER_MODE); } 

    /**
     * @brief Enables PCM slave mode 
     */
    void enablePCMSlaveMode() { writeRegister8(ES9039Q2M_REG_INPUT_SELECTION, readRegister8(ES9039Q2M_REG_INPUT_SELECTION) & ~ES9039Q2M_BIT_PCM_MASTER_MODE); }

    /**
     * @brief Enables DSD master mode 
     */
    void enableDSDMasterMode() { writeRegister8(ES9039Q2M_REG_INPUT_SELECTION, readRegister8(ES9039Q2M_REG_INPUT_SELECTION) | ES9039Q2M_BIT_DSD_MASTER_MODE); } 

    /**
     * @brief Enables DSD slave mode 
     */
    void enableDSDSlaveMode() { writeRegister8(ES9039Q2M_REG_INPUT_SELECTION, readRegister8(ES9039Q2M_REG_INPUT_SELECTION) & ~ES9039Q2M_BIT_DSD_MASTER_MODE); }


    // Master encoder configuration functions
    /**
     * @brief Set master encoder configuration register
     * @param config Configuration value
     */
    void setMasterEncoderConfig(uint8_t config) { writeRegister8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG, config); }
    
    /**
     * @brief Get master encoder configuration register
     * @return uint8_t Current configuration value
     */
    uint8_t getMasterEncoderConfig() { return readRegister8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG); }
    
    /**
     * @brief Enable TDM resynchronization
     */
    void enableTDMResync() { writeRegisterMasked8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG, ES9039Q2M_BIT_TDM_RESYNC, ES9039Q2M_BIT_TDM_RESYNC); }
    
    /**
     * @brief Disable TDM resynchronization
     */
    void disableTDMResync() { writeRegisterMasked8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG, ES9039Q2M_BIT_TDM_RESYNC, 0); }
    
    /**
     * @brief Check if TDM resynchronization is enabled
     * @return true if enabled, false otherwise
     */
    bool isTDMResyncEnabled() { return (readRegister8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG) & ES9039Q2M_BIT_TDM_RESYNC) != 0; }
    
    /**
     * @brief Enable BCK (bit clock) inversion
     */
    void enableBCKInvert() { writeRegisterMasked8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG, ES9039Q2M_BIT_BCK_INV, ES9039Q2M_BIT_BCK_INV); }
    
    /**
     * @brief Disable BCK inversion
     */
    void disableBCKInvert() { writeRegisterMasked8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG, ES9039Q2M_BIT_BCK_INV, 0); }
    
    /**
     * @brief Check if BCK inversion is enabled
     * @return true if enabled, false otherwise
     */
    bool isBCKInvertEnabled() { return (readRegister8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG) & ES9039Q2M_BIT_BCK_INV) != 0; }
    
    /**
     * @brief Set master mode frame length
     * @param length Frame length value
     */
    void setMasterFrameLength(uint8_t length) { writeRegisterMasked8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG, ES9039Q2M_BIT_MASTER_FRAME_LENGTH_MASK, length << 3); }
    
    /**
     * @brief Get master mode frame length
     * @return uint8_t Frame length value
     */
    uint8_t getMasterFrameLength() { return (readRegister8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG) & ES9039Q2M_BIT_MASTER_FRAME_LENGTH_MASK) >> 3; }
    
    /**
     * @brief Enable master mode WS (word select) pulse
     */
    void enableMasterWSPulse() { writeRegisterMasked8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG, ES9039Q2M_BIT_MASTER_WS_PULSE, ES9039Q2M_BIT_MASTER_WS_PULSE); }
    
    /**
     * @brief Disable master mode WS pulse
     */
    void disableMasterWSPulse() { writeRegisterMasked8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG, ES9039Q2M_BIT_MASTER_WS_PULSE, 0); }
    
    /**
     * @brief Check if master mode WS pulse is enabled
     * @return true if enabled, false otherwise
     */
    bool isMasterWSPulseEnabled() { return (readRegister8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG) & ES9039Q2M_BIT_MASTER_WS_PULSE) != 0; }
    
    /**
     * @brief Enable master mode WS inversion
     */
    void enableMasterWSInvert() { writeRegisterMasked8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG, ES9039Q2M_BIT_MASTER_WS_INV, ES9039Q2M_BIT_MASTER_WS_INV); }
    
    /**
     * @brief Disable master mode WS inversion
     */
    void disableMasterWSInvert() { writeRegisterMasked8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG, ES9039Q2M_BIT_MASTER_WS_INV, 0); }
    
    /**
     * @brief Check if master mode WS inversion is enabled
     * @return true if enabled, false otherwise
     */
    bool isMasterWSInvertEnabled() { return (readRegister8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG) & ES9039Q2M_BIT_MASTER_WS_INV) != 0; }
    
    /**
     * @brief Enable master mode BCK inversion
     */
    void enableMasterBCKInvert() { writeRegisterMasked8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG, ES9039Q2M_BIT_MASTER_BCK_INV, ES9039Q2M_BIT_MASTER_BCK_INV); }
    
    /**
     * @brief Disable master mode BCK inversion
     */
    void disableMasterBCKInvert() { writeRegisterMasked8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG, ES9039Q2M_BIT_MASTER_BCK_INV, 0); }
    
    /**
     * @brief Check if master mode BCK inversion is enabled
     * @return true if enabled, false otherwise
     */
    bool isMasterBCKInvertEnabled() { return (readRegister8(ES9039Q2M_REG_MASTER_ENCODER_CONFIG) & ES9039Q2M_BIT_MASTER_BCK_INV) != 0; }

    // TDM configuration functions
    /**
     * @brief Set TDM channel configuration
     * @param config TDM channel value
     */
    void setTDMChannel(uint8_t config) { writeRegister8(ES9039Q2M_REG_TDM_CONFIG, config & 0x1f); }
    
    /**
     * @brief Get TDM channel configuration
     * @return uint8_t Current TDM channel value
     */
    uint8_t getTDMChannel() { return readRegister8(ES9039Q2M_REG_TDM_CONFIG) & 0x1f; }

    // TDM1 configuration functions
    /**
     * @brief Enable TDM1 left-justified mode
     */
    void enableTDM1LJMode() { writeRegisterMasked8(ES9039Q2M_REG_TDM_CONFIG1, ES9039Q2M_BIT_TDM_LJ_MODE, ES9039Q2M_BIT_TDM_LJ_MODE); }
    
    /**
     * @brief Disable TDM1 left-justified mode
     */
    void disableTDM1LJMode() { writeRegisterMasked8(ES9039Q2M_REG_TDM_CONFIG1, ES9039Q2M_BIT_TDM_LJ_MODE, 0); }
    
    /**
     * @brief Check if TDM1 left-justified mode is enabled
     * @return true if enabled, false otherwise
     */
    bool isTDM1LJModeEnabled() { return (readRegister8(ES9039Q2M_REG_TDM_CONFIG1) & ES9039Q2M_BIT_TDM_LJ_MODE) != 0; }
    
    /**
     * @brief Enable TDM1 valid edge detection
     */
    void enableTDM1ValidEdge() { writeRegisterMasked8(ES9039Q2M_REG_TDM_CONFIG1, ES9039Q2M_BIT_TDM_VALID_EDGE, ES9039Q2M_BIT_TDM_VALID_EDGE); }
    
    /**
     * @brief Disable TDM1 valid edge detection
     */
    void disableTDM1ValidEdge() { writeRegisterMasked8(ES9039Q2M_REG_TDM_CONFIG1, ES9039Q2M_BIT_TDM_VALID_EDGE, 0); }
    
    /**
     * @brief Check if TDM1 valid edge detection is enabled
     * @return true if enabled, false otherwise
     */
    bool isTDM1ValidEdgeEnabled() { return (readRegister8(ES9039Q2M_REG_TDM_CONFIG1) & ES9039Q2M_BIT_TDM_VALID_EDGE) != 0; }
    
    /**
     * @brief Enable TDM1 daisy chain mode
     */
    void enableTDM1DaisyChain() { writeRegisterMasked8(ES9039Q2M_REG_TDM_CONFIG1, ES9039Q2M_BIT_TDM_DAISY_CHAIN, ES9039Q2M_BIT_TDM_DAISY_CHAIN); }
    
    /**
     * @brief Disable TDM1 daisy chain mode
     */
    void disableTDM1DaisyChain() { writeRegisterMasked8(ES9039Q2M_REG_TDM_CONFIG1, ES9039Q2M_BIT_TDM_DAISY_CHAIN, 0); }
    
    /**
     * @brief Check if TDM1 daisy chain mode is enabled
     * @return true if enabled, false otherwise
     */
    bool isTDM1DaisyChainEnabled() { return (readRegister8(ES9039Q2M_REG_TDM_CONFIG1) & ES9039Q2M_BIT_TDM_DAISY_CHAIN) != 0; }

    // TDM second set configuration functions
    /**
     * @brief Set TDM latch adjust value
     * @param adjust Latch adjust value
     */
    void setTDMLatchAdjust(uint8_t adjust) { writeRegisterMasked8(ES9039Q2M_REG_TDM_CONFIG1, ES9039Q2M_BIT_TDM_LATCH_ADJUST_MASK, adjust & ES9039Q2M_BIT_TDM_LATCH_ADJUST_MASK); }
    
    /**
     * @brief Get TDM latch adjust value
     * @return uint8_t Current latch adjust value
     */
    uint8_t getTDMLatchAdjust() { return readRegister8(ES9039Q2M_REG_TDM_CONFIG1) & ES9039Q2M_BIT_TDM_LATCH_ADJUST_MASK; }
    
    /**
     * @brief Set TDM bit width
     * @param width Bit width setting
     */
    void setTDMBitWidth(uint8_t width) { writeRegisterMasked8(ES9039Q2M_REG_TDM_CONFIG1, ES9039Q2M_BIT_TDM_BIT_WIDTH_MASK, width << ES9039Q2M_BIT_TDM_BIT_WIDTH_SHIFT); }
    
    /**
     * @brief Get TDM bit width
     * @return uint8_t Current bit width setting
     */
    uint8_t getTDMBitWidth() { return (readRegister8(ES9039Q2M_REG_TDM_CONFIG1) & ES9039Q2M_BIT_TDM_BIT_WIDTH_MASK) >> ES9039Q2M_BIT_TDM_BIT_WIDTH_SHIFT; }

    /**
     * @brief Get TDM configuration 1 register
     * @return uint8_t Current TDM configuration 1 value
     */
    uint8_t getTDMConfig1() { return readRegister8(ES9039Q2M_REG_TDM_CONFIG1); }

    // BCK/WS Monitor configuration functions
    /**
     * @brief Enable word select monitor
     */
    void enableWSMonitor() { writeRegisterMasked8(ES9039Q2M_REG_BCK_WS_MONITOR_CONFIG, ES9039Q2M_BIT_ENABLE_WS_MONITOR, ES9039Q2M_BIT_ENABLE_WS_MONITOR); }
    
    /**
     * @brief Disable word select monitor
     */
    void disableWSMonitor() { writeRegisterMasked8(ES9039Q2M_REG_BCK_WS_MONITOR_CONFIG, ES9039Q2M_BIT_ENABLE_WS_MONITOR, 0); }
    
    /**
     * @brief Check if word select monitor is enabled
     * @return true if enabled, false otherwise
     */
    bool isWSMonitorEnabled() { return (readRegister8(ES9039Q2M_REG_BCK_WS_MONITOR_CONFIG) & ES9039Q2M_BIT_ENABLE_WS_MONITOR) != 0; }

    /**
     * @brief Enable bit clock monitor
     */
    void enableBCKMonitor() { writeRegisterMasked8(ES9039Q2M_REG_BCK_WS_MONITOR_CONFIG, ES9039Q2M_BIT_ENABLE_BCK_MONITOR, ES9039Q2M_BIT_ENABLE_BCK_MONITOR); }
    
    /**
     * @brief Disable bit clock monitor
     */
    void disableBCKMonitor() { writeRegisterMasked8(ES9039Q2M_REG_BCK_WS_MONITOR_CONFIG, ES9039Q2M_BIT_ENABLE_BCK_MONITOR, 0); }
    
    /**
     * @brief Check if bit clock monitor is enabled
     * @return true if enabled, false otherwise
     */
    bool isBCKMonitorEnabled() { return (readRegister8(ES9039Q2M_REG_BCK_WS_MONITOR_CONFIG) & ES9039Q2M_BIT_ENABLE_BCK_MONITOR) != 0; }
    
    /**
     * @brief Disable DSD DC blocking
     */
    void disableDSDDC() { writeRegisterMasked8(ES9039Q2M_REG_BCK_WS_MONITOR_CONFIG, ES9039Q2M_BIT_DISABLE_DSD_DC, ES9039Q2M_BIT_DISABLE_DSD_DC); }
    
    /**
     * @brief Enable DSD DC blocking
     */
    void enableDSDDC() { writeRegisterMasked8(ES9039Q2M_REG_BCK_WS_MONITOR_CONFIG, ES9039Q2M_BIT_DISABLE_DSD_DC, 0); }
    
    /**
     * @brief Check if DSD DC blocking is disabled
     * @return true if disabled, false if enabled
     */
    bool isDSDDCDisabled() { return (readRegister8(ES9039Q2M_REG_BCK_WS_MONITOR_CONFIG) & ES9039Q2M_BIT_DISABLE_DSD_DC) != 0; }

    /**
     * @brief Disable DSD automatic mute
     */
    void disableDSDMute() { writeRegisterMasked8(ES9039Q2M_REG_BCK_WS_MONITOR_CONFIG, ES9039Q2M_BIT_DISABLE_DSD_MUTE, ES9039Q2M_BIT_DISABLE_DSD_MUTE); }
    
    /**
     * @brief Enable DSD automatic mute
     */
    void enableDSDMute() { writeRegisterMasked8(ES9039Q2M_REG_BCK_WS_MONITOR_CONFIG, ES9039Q2M_BIT_DISABLE_DSD_MUTE, 0); }
    
    /**
     * @brief Check if DSD automatic mute is disabled
     * @return true if disabled, false if enabled
     */
    bool isDSDMuteDisabled() { return (readRegister8(ES9039Q2M_REG_BCK_WS_MONITOR_CONFIG) & ES9039Q2M_BIT_DISABLE_DSD_MUTE) != 0; }
    
    /**
     * @brief Disable PCM DC blocking
     */
    void disablePCMDC() { writeRegisterMasked8(ES9039Q2M_REG_BCK_WS_MONITOR_CONFIG, ES9039Q2M_BIT_DISABLE_PCM_DC, ES9039Q2M_BIT_DISABLE_PCM_DC); }
    
    /**
     * @brief Enable PCM DC blocking
     */
    void enablePCMDC() { writeRegisterMasked8(ES9039Q2M_REG_BCK_WS_MONITOR_CONFIG, ES9039Q2M_BIT_DISABLE_PCM_DC, 0); }
    
    /**
     * @brief Check if PCM DC blocking is disabled
     * @return true if disabled, false if enabled
     */
    bool isPCMDCDisabled() { return (readRegister8(ES9039Q2M_REG_BCK_WS_MONITOR_CONFIG) & ES9039Q2M_BIT_DISABLE_PCM_DC) != 0; }

    /**
     * @brief Get BCK/WS monitor configuration register
     * @return uint8_t Current BCK/WS monitor configuration value
     */
    uint8_t getBCKWSMonitorConfig() { return readRegister8(ES9039Q2M_REG_BCK_WS_MONITOR_CONFIG); }

    // CH1 slot configuration functions
    /**
     * @brief Set channel 1 slot source
     * @param source Slot source value
     */
    void setCH1SlotSource(uint8_t source) { writeRegisterMasked8(ES9039Q2M_REG_BCK_CH1_SLOT_CONFIG, ES9039Q2M_BIT_DSD_SLOT_SOURCE_MASK, (source & (ES9039Q2M_BIT_DSD_SLOT_SOURCE_MASK >> ES9039Q2M_BIT_DSD_SLOT_SOURCE_SHIFT)) << ES9039Q2M_BIT_DSD_SLOT_SOURCE_SHIFT); }
    
    /**
     * @brief Get channel 1 slot source
     * @return uint8_t Current slot source value
     */
    uint8_t getCH1SlotSource() { return (readRegister8(ES9039Q2M_REG_BCK_CH1_SLOT_CONFIG) & ES9039Q2M_BIT_DSD_SLOT_SOURCE_MASK) >> ES9039Q2M_BIT_DSD_SLOT_SOURCE_SHIFT ; }
    
    /**
     * @brief Set channel 1 TDM slot
     * @param slot TDM slot number
     */
    void setCH1TDMSlot(uint8_t slot) { writeRegisterMasked8(ES9039Q2M_REG_BCK_CH1_SLOT_CONFIG, ES9039Q2M_BIT_TDM_SLOT_SEL_MASK, slot & ES9039Q2M_BIT_TDM_SLOT_SEL_MASK); }
    
    /**
     * @brief Get channel 1 TDM slot
     * @return uint8_t Current TDM slot number
     */
    uint8_t getCH1TDMSlot() { return readRegister8(ES9039Q2M_REG_BCK_CH1_SLOT_CONFIG) & ES9039Q2M_BIT_TDM_SLOT_SEL_MASK; }

    /**
     * @brief Get BCK channel 1 slot configuration register
     * @return uint8_t Current BCK channel 1 slot configuration value
     */
    uint8_t getBCKCH1SlotConfig() { return readRegister8(ES9039Q2M_REG_BCK_CH1_SLOT_CONFIG); }

    // CH2 slot configuration functions
    /**
     * @brief Set channel 2 slot source
     * @param source Slot source value
     */
    void setCH2SlotSource(uint8_t source) { writeRegisterMasked8(ES9039Q2M_REG_BCK_CH2_SLOT_CONFIG, ES9039Q2M_BIT_DSD_SLOT_SOURCE_MASK, (source & (ES9039Q2M_BIT_DSD_SLOT_SOURCE_MASK >> ES9039Q2M_BIT_DSD_SLOT_SOURCE_SHIFT)) << ES9039Q2M_BIT_DSD_SLOT_SOURCE_SHIFT); }
    
    /**
     * @brief Get channel 2 slot source
     * @return uint8_t Current slot source value
     */
    uint8_t getCH2SlotSource() { return (readRegister8(ES9039Q2M_REG_BCK_CH2_SLOT_CONFIG) & ES9039Q2M_BIT_DSD_SLOT_SOURCE_MASK) >> ES9039Q2M_BIT_DSD_SLOT_SOURCE_SHIFT ; }
    
    /**
     * @brief Set channel 2 TDM slot
     * @param slot TDM slot number
     */
    void setCH2TDMSlot(uint8_t slot) { writeRegisterMasked8(ES9039Q2M_REG_BCK_CH2_SLOT_CONFIG, ES9039Q2M_BIT_TDM_SLOT_SEL_MASK, slot & ES9039Q2M_BIT_TDM_SLOT_SEL_MASK); }
    
    /**
     * @brief Get channel 2 TDM slot
     * @return uint8_t Current TDM slot number
     */
    uint8_t getCH2TDMSlot() { return readRegister8(ES9039Q2M_REG_BCK_CH2_SLOT_CONFIG) & ES9039Q2M_BIT_TDM_SLOT_SEL_MASK; }

    /**
     * @brief Get BCK channel 2 slot configuration register
     * @return uint8_t Current BCK channel 2 slot configuration value
     */
    uint8_t getBCKCH2SlotConfig() { return readRegister8(ES9039Q2M_REG_BCK_CH2_SLOT_CONFIG); }

    // Volume control functions
    /**
     * @brief Set channel 1 volume level
     * @param volume Volume value (0-255, 0 = max, 255 = mute)
     */
    void setCH1Volume(uint8_t volume) { writeRegister8(ES9039Q2M_REG_VOLUME_CH1, volume); }
    
    /**
     * @brief Get channel 1 volume level
     * @return uint8_t Current volume value
     */
    uint8_t getCH1Volume() { return readRegister8(ES9039Q2M_REG_VOLUME_CH1); }
    
    /**
     * @brief Set channel 2 volume level
     * @param volume Volume value (0-255, 0 = max, 255 = mute)
     */
    void setCH2Volume(uint8_t volume) { writeRegister8(ES9039Q2M_REG_VOLUME_CH2, volume); }
    
    /**
     * @brief Get channel 2 volume level
     * @return uint8_t Current volume value
     */
    uint8_t getCH2Volume() { return readRegister8(ES9039Q2M_REG_VOLUME_CH2); }

    // Volume rate control functions
    /**
     * @brief Set volume ramp-up rate
     * @param rate Volume ramp-up rate value
     */
    void setVolumeRateUp(uint8_t rate) { writeRegister8(ES9039Q2M_REG_VOL_RATE_UP, rate); }
    
    /**
     * @brief Get volume ramp-up rate
     * @return uint8_t Current ramp-up rate value
     */
    uint8_t getVolumeRateUp() { return readRegister8(ES9039Q2M_REG_VOL_RATE_UP); }
    
    /**
     * @brief Set volume ramp-down rate
     * @param rate Volume ramp-down rate value
     */
    void setVolumeRateDown(uint8_t rate) { writeRegister8(ES9039Q2M_REG_VOL_RATE_DOWN, rate); }
    
    /**
     * @brief Get volume ramp-down rate
     * @return uint8_t Current ramp-down rate value
     */
    uint8_t getVolumeRateDown() { return readRegister8(ES9039Q2M_REG_VOL_RATE_DOWN); }
    
    /**
     * @brief Set fast volume ramp rate
     * @param rate Fast volume ramp rate value
     */
    void setVolumeRateFast(uint8_t rate) { writeRegister8(ES9039Q2M_REG_VOL_RATE_FAST, rate); }
    
    /**
     * @brief Get fast volume ramp rate
     * @return uint8_t Current fast ramp rate value
     */
    uint8_t getVolumeRateFast() { return readRegister8(ES9039Q2M_REG_VOL_RATE_FAST); }

    // DAC mute functions
    /**
     * @brief Mute channel 1 DAC output
     */
    void muteCh1() { writeRegisterMasked8(ES9039Q2M_REG_DAC_MUTE, ES9039Q2M_BIT_DAC_MUTE_CH1, ES9039Q2M_BIT_DAC_MUTE_CH1); }
    
    /**
     * @brief Unmute channel 1 DAC output
     */
    void unmuteCh1() { writeRegisterMasked8(ES9039Q2M_REG_DAC_MUTE, ES9039Q2M_BIT_DAC_MUTE_CH1, 0); }
    
    /**
     * @brief Check if channel 1 DAC is muted
     * @return true if muted, false otherwise
     */
    bool isDACCh1Muted() { return (readRegister8(ES9039Q2M_REG_DAC_MUTE) & ES9039Q2M_BIT_DAC_MUTE_CH1) != 0; }
    
    /**
     * @brief Mute channel 2 DAC output
     */
    void muteCh2() { writeRegisterMasked8(ES9039Q2M_REG_DAC_MUTE, ES9039Q2M_BIT_DAC_MUTE_CH2, ES9039Q2M_BIT_DAC_MUTE_CH2); }
    
    /**
     * @brief Unmute channel 2 DAC output
     */
    void unmuteCh2() { writeRegisterMasked8(ES9039Q2M_REG_DAC_MUTE, ES9039Q2M_BIT_DAC_MUTE_CH2, 0); }

    /**
     * Mute both DAC channels
     */
    void mute() { writeRegisterMasked8(ES9039Q2M_REG_DAC_MUTE, ES9039Q2M_BIT_DAC_MUTE_CH1 | ES9039Q2M_BIT_DAC_MUTE_CH2, ES9039Q2M_BIT_DAC_MUTE_CH1 | ES9039Q2M_BIT_DAC_MUTE_CH2); }

    /**
     * Unmute both DAC channels
     */
    void unmute() { writeRegisterMasked8(ES9039Q2M_REG_DAC_MUTE, ES9039Q2M_BIT_DAC_MUTE_CH1 | ES9039Q2M_BIT_DAC_MUTE_CH2, 0); }
    
    /**
     * @brief Check if channel 2 DAC is muted
     * @return true if muted, false otherwise
     */
    bool isDACCh2Muted() { return (readRegister8(ES9039Q2M_REG_DAC_MUTE) & ES9039Q2M_BIT_DAC_MUTE_CH2) != 0; }

    /**
     * @brief Get DAC mute register
     * @return uint8_t Current DAC mute value
     */
    uint8_t getDACMute() { return readRegister8(ES9039Q2M_REG_DAC_MUTE); }

    // DAC invert functions
    /**
     * @brief Invert channel 1 DAC output polarity
     */
    void invertDACCh1() { writeRegisterMasked8(ES9039Q2M_REG_DAC_INVERT, ES9039Q2M_BIT_DAC_INVERT_CH1, ES9039Q2M_BIT_DAC_INVERT_CH1); }
    
    /**
     * @brief Un-invert channel 1 DAC output polarity
     */
    void uninvertDACCh1() { writeRegisterMasked8(ES9039Q2M_REG_DAC_INVERT, ES9039Q2M_BIT_DAC_INVERT_CH1, 0); }
    
    /**
     * @brief Check if channel 1 DAC output is inverted
     * @return true if inverted, false otherwise
     */
    bool isDACCh1Inverted() { return (readRegister8(ES9039Q2M_REG_DAC_INVERT) & ES9039Q2M_BIT_DAC_INVERT_CH1) != 0; }
    
    /**
     * @brief Invert channel 2 DAC output polarity
     */
    void invertDACCh2() { writeRegisterMasked8(ES9039Q2M_REG_DAC_INVERT, ES9039Q2M_BIT_DAC_INVERT_CH2, ES9039Q2M_BIT_DAC_INVERT_CH2); }
    
    /**
     * @brief Un-invert channel 2 DAC output polarity
     */
    void uninvertDACCh2() { writeRegisterMasked8(ES9039Q2M_REG_DAC_INVERT, ES9039Q2M_BIT_DAC_INVERT_CH2, 0); }
    
    /**
     * @brief Check if channel 2 DAC output is inverted
     * @return true if inverted, false otherwise
     */
    bool isDACCh2Inverted() { return (readRegister8(ES9039Q2M_REG_DAC_INVERT) & ES9039Q2M_BIT_DAC_INVERT_CH2) != 0; }

    /**
     * @brief Get DAC invert register
     * @return uint8_t Current DAC invert value
     */
    uint8_t getDACInvert() { return readRegister8(ES9039Q2M_REG_DAC_INVERT); }

    // Filter shape functions
    /**
     * @brief Set digital filter shape
     * @param shape Filter shape value (0-7)
     */
    void setFilterShape(uint8_t shape) { writeRegisterMasked8(ES9039Q2M_REG_FILTER_SHAPE, ES9039Q2M_BIT_FILTER_SHAPE_MASK, shape & ES9039Q2M_BIT_FILTER_SHAPE_MASK); }
    
    /**
     * @brief Get digital filter shape
     * @return uint8_t Current filter shape value
     */
    uint8_t getFilterShape() { return readRegister8(ES9039Q2M_REG_FILTER_SHAPE) & ES9039Q2M_BIT_FILTER_SHAPE_MASK; }

    // IIR Bandwidth and SPDIF Selection functions
    /**
     * @brief Set IIR filter bandwidth
     * @param bw Bandwidth value
     */
    void setIIRBandwidth(uint8_t bw) { writeRegisterMasked8(ES9039Q2M_REG_IIR_BW_SPDIF_SEL, ES9039Q2M_BIT_IIR_BW_MASK, bw & ES9039Q2M_BIT_IIR_BW_MASK); }
    
    /**
     * @brief Get IIR filter bandwidth
     * @return uint8_t Current bandwidth value
     */
    uint8_t getIIRBandwidth() { return readRegister8(ES9039Q2M_REG_IIR_BW_SPDIF_SEL) & ES9039Q2M_BIT_IIR_BW_MASK; }
    
    /**
     * @brief Set S/PDIF input selection
     * @param sel S/PDIF selection value
     */
    void setSPDIFSelection(uint8_t sel) { writeRegisterMasked8(ES9039Q2M_REG_IIR_BW_SPDIF_SEL, ES9039Q2M_BIT_SPDIF_SEL_MASK, (sel << ES9039Q2M_BIT_SPDIF_SEL_SHIFT) & ES9039Q2M_BIT_SPDIF_SEL_MASK); }
    
    /**
     * @brief Get S/PDIF input selection
     * @return uint8_t Current S/PDIF selection value
     */
    uint8_t getSPDIFSelection() { return (readRegister8(ES9039Q2M_REG_IIR_BW_SPDIF_SEL) & ES9039Q2M_BIT_SPDIF_SEL_MASK) >> ES9039Q2M_BIT_SPDIF_SEL_SHIFT; }
    
    /**
     * @brief Enable volume hold
     * @param hold true to enable, false to disable
     */
    void enableVolumeHold(bool hold) { writeRegisterMasked8(ES9039Q2M_REG_IIR_BW_SPDIF_SEL, ES9039Q2M_BIT_VOLUME_HOLD, hold ? ES9039Q2M_BIT_VOLUME_HOLD : 0); }
    
    /**
     * @brief Disable volume hold
     */
    void disableVolumeHold() { writeRegisterMasked8(ES9039Q2M_REG_IIR_BW_SPDIF_SEL, ES9039Q2M_BIT_VOLUME_HOLD, 0); }
    
    /**
     * @brief Check if volume hold is enabled
     * @return true if enabled, false otherwise
     */
    bool isVolumeHold() { return (readRegister8(ES9039Q2M_REG_IIR_BW_SPDIF_SEL) & ES9039Q2M_BIT_VOLUME_HOLD) != 0; }

    /**
     * @brief Get IIR bandwidth and SPDIF selection register
     * @return uint8_t Current IIR bandwidth and SPDIF selection value
     */
    uint8_t getIIRBWSPDIFSel() { return readRegister8(ES9039Q2M_REG_IIR_BW_SPDIF_SEL); }

    // DAC Path Config functions
    /**
     * @brief Enable IIR filter bypass
     */
    void enableBypassIIR() { writeRegisterMasked8(ES9039Q2M_REG_DAC_PATH_CONFIG, ES9039Q2M_BIT_BYPASS_IIR, ES9039Q2M_BIT_BYPASS_IIR); }
    
    /**
     * @brief Disable IIR filter bypass
     */
    void disableBypassIIR() { writeRegisterMasked8(ES9039Q2M_REG_DAC_PATH_CONFIG, ES9039Q2M_BIT_BYPASS_IIR, 0); }
    
    /**
     * @brief Enable 4X FIR filter bypass
     */
    void enableBypassFIR4X() { writeRegisterMasked8(ES9039Q2M_REG_DAC_PATH_CONFIG, ES9039Q2M_BIT_BYPASS_FIR_4X, ES9039Q2M_BIT_BYPASS_FIR_4X); }
    
    /**
     * @brief Disable 4X FIR filter bypass
     */
    void disableBypassFIR4X() { writeRegisterMasked8(ES9039Q2M_REG_DAC_PATH_CONFIG, ES9039Q2M_BIT_BYPASS_FIR_4X, 0); }
    
    /**
     * @brief Enable 2X FIR filter bypass
     */
    void enableBypassFIR2X() { writeRegisterMasked8(ES9039Q2M_REG_DAC_PATH_CONFIG, ES9039Q2M_BIT_BYPASS_FIR_2X, ES9039Q2M_BIT_BYPASS_FIR_2X); }
    
    /**
     * @brief Disable 2X FIR filter bypass
     */
    void disableBypassFIR2X() { writeRegisterMasked8(ES9039Q2M_REG_DAC_PATH_CONFIG, ES9039Q2M_BIT_BYPASS_FIR_2X, 0); }
    
    /**
     * @brief Check if IIR filter bypass is enabled
     * @return true if bypass enabled, false otherwise
     */
    bool isBypassIIREnabled() { return (readRegister8(ES9039Q2M_REG_DAC_PATH_CONFIG) & ES9039Q2M_BIT_BYPASS_IIR) != 0; }
    
    /**
     * @brief Check if 4X FIR filter bypass is enabled
     * @return true if bypass enabled, false otherwise
     */
    bool isBypassFIR4XEnabled() { return (readRegister8(ES9039Q2M_REG_DAC_PATH_CONFIG) & ES9039Q2M_BIT_BYPASS_FIR_4X) != 0; }
    
    /**
     * @brief Check if 2X FIR filter bypass is enabled
     * @return true if bypass enabled, false otherwise
     */
    bool isBypassFIR2XEnabled() { return (readRegister8(ES9039Q2M_REG_DAC_PATH_CONFIG) & ES9039Q2M_BIT_BYPASS_FIR_2X) != 0; }

    /**
     * @brief Get DAC path configuration register
     * @return uint8_t Current DAC path configuration value
     */
    uint8_t getDACPathConfig() { return readRegister8(ES9039Q2M_REG_DAC_PATH_CONFIG); }

    // THD C2 functions
    /**
     * @brief Set THD 2nd harmonic compensation for channel 1
     * @param value Compensation value
     */
    void setTHDC2Ch1(uint16_t value) { writeRegister16(ES9039Q2M_REG_THD_C2_CH1, value); }
    
    /**
     * @brief Get THD 2nd harmonic compensation for channel 1
     * @return uint16_t Current compensation value
     */
    uint16_t getTHDC2Ch1() { return readRegister16(ES9039Q2M_REG_THD_C2_CH1); }
    
    /**
     * @brief Set THD 2nd harmonic compensation for channel 2
     * @param value Compensation value
     */
    void setTHDC2Ch2(uint16_t value) { writeRegister16(ES9039Q2M_REG_THD_C2_CH2, value); }
    
    /**
     * @brief Get THD 2nd harmonic compensation for channel 2
     * @return uint16_t Current compensation value
     */
    uint16_t getTHDC2Ch2() { return readRegister16(ES9039Q2M_REG_THD_C2_CH2); }

    // THD C3 functions
    /**
     * @brief Set THD 3rd harmonic compensation for channel 1
     * @param value Compensation value
     */
    void setTHDC3Ch1(uint16_t value) { writeRegister16(ES9039Q2M_REG_THD_C3_CH1, value); }
    
    /**
     * @brief Get THD 3rd harmonic compensation for channel 1
     * @return uint16_t Current compensation value
     */
    uint16_t getTHDC3Ch1() { return readRegister16(ES9039Q2M_REG_THD_C3_CH1); }
    
    /**
     * @brief Set THD 3rd harmonic compensation for channel 2
     * @param value Compensation value
     */
    void setTHDC3Ch2(uint16_t value) { writeRegister16(ES9039Q2M_REG_THD_C3_CH2, value); }
    
    /**
     * @brief Get THD 3rd harmonic compensation for channel 2
     * @return uint16_t Current compensation value
     */
    uint16_t getTHDC3Ch2() { return readRegister16(ES9039Q2M_REG_THD_C3_CH2); }

    // Automute functions
    /**
     * @brief Enable automute for channel 1
     */
    void enableAutomuteCh1() { writeRegisterMasked8(ES9039Q2M_AUTOMUTE_ENABLE, ES9039Q2M_BIT_EN_CH1, ES9039Q2M_BIT_EN_CH1); }
    
    /**
     * @brief Disable automute for channel 1
     */
    void disableAutomuteCh1() { writeRegisterMasked8(ES9039Q2M_AUTOMUTE_ENABLE, ES9039Q2M_BIT_EN_CH1, 0); }
    
    /**
     * @brief Check if automute is enabled for channel 1
     * @return true if enabled, false otherwise
     */
    bool isAutomuteCh1Enabled() { return (readRegister8(ES9039Q2M_AUTOMUTE_ENABLE) & ES9039Q2M_BIT_EN_CH1) != 0; }

    /**
     * @brief Enable automute for channel 2
     */
    void enableAutomuteCh2() { writeRegisterMasked8(ES9039Q2M_AUTOMUTE_ENABLE, ES9039Q2M_BIT_EN_CH2, ES9039Q2M_BIT_EN_CH2); }
    
    /**
     * @brief Disable automute for channel 2
     */
    void disableAutomuteCh2() { writeRegisterMasked8(ES9039Q2M_AUTOMUTE_ENABLE, ES9039Q2M_BIT_EN_CH2, 0); }

    /**
     * @brief Enable automute for both channels
     */
    void enableAutomute() { writeRegisterMasked8(ES9039Q2M_AUTOMUTE_ENABLE, ES9039Q2M_BIT_EN_CH1 | ES9039Q2M_BIT_EN_CH2, ES9039Q2M_BIT_EN_CH1 | ES9039Q2M_BIT_EN_CH2); }

    /**
     * @brief Disable automute for both channels
     */ 
    void disableAutomute() { writeRegisterMasked8(ES9039Q2M_AUTOMUTE_ENABLE, ES9039Q2M_BIT_EN_CH1 | ES9039Q2M_BIT_EN_CH2, 0); }
    
    /**
     * @brief Check if automute is enabled for channel 2
     * @return true if enabled, false otherwise
     */
    bool isAutomuteCh2Enabled() { return (readRegister8(ES9039Q2M_AUTOMUTE_ENABLE) & ES9039Q2M_BIT_EN_CH2) != 0; }

    /**
     * @brief Get automute enable register
     * @return uint8_t Current automute enable value
     */
    uint8_t getAutomuteEnable() { return readRegister8(ES9039Q2M_AUTOMUTE_ENABLE); }

    // Automute time functions
    /**
     * @brief Set automute time threshold
     * @param value Automute time value
     */
    void setAutomuteTime(uint16_t value) { writeRegisterMasked16(ES9039Q2M_AUTOMUTE_TIME, ES9039Q2M_BIT_AUTOMUTE_TIME_MASK, value); }
    
    /**
     * @brief Get automute time threshold
     * @return uint16_t Current automute time value
     */
    uint16_t getAutomuteTime() { return readRegister16(ES9039Q2M_AUTOMUTE_TIME) & ES9039Q2M_BIT_AUTOMUTE_TIME_MASK; }
    
    /**
     * @brief Enable external dynamic element matching
     */
    void enableExternalDem() { writeRegisterMasked16(ES9039Q2M_AUTOMUTE_TIME, ES9039Q2M_BIT_EXTERNAL_DEM, ES9039Q2M_BIT_EXTERNAL_DEM); }
    
    /**
     * @brief Disable external dynamic element matching
     */
    void disableExternalDem() { writeRegisterMasked16(ES9039Q2M_AUTOMUTE_TIME, ES9039Q2M_BIT_EXTERNAL_DEM, 0); }
    
    /**
     * @brief Check if external DEM is enabled
     * @return true if enabled, false otherwise
     */
    bool isExternalDemEnabled() { return (readRegister16(ES9039Q2M_AUTOMUTE_TIME) & ES9039Q2M_BIT_EXTERNAL_DEM) != 0; }
    
    /**
     * @brief Enable external DEM interrupt enable
     */
    void enableExternalDemIE() { writeRegisterMasked16(ES9039Q2M_AUTOMUTE_TIME, ES9039Q2M_BIT_EXTERNAL_DEM_IE, ES9039Q2M_BIT_EXTERNAL_DEM_IE); }
    
    /**
     * @brief Disable external DEM interrupt enable
     */
    void disableExternalDemIE() { writeRegisterMasked16(ES9039Q2M_AUTOMUTE_TIME, ES9039Q2M_BIT_EXTERNAL_DEM_IE, 0); }
    
    /**
     * @brief Check if external DEM interrupt enable is enabled
     * @return true if enabled, false otherwise
     */
    bool isExternalDemIEEnabled() { return (readRegister16(ES9039Q2M_AUTOMUTE_TIME) & ES9039Q2M_BIT_EXTERNAL_DEM_IE) != 0; }

    /**
     * @brief Get automute time register
     * @return uint16_t Current automute time register value
     */
    uint16_t getAutomuteTimeRegister() { return readRegister16(ES9039Q2M_AUTOMUTE_TIME); }

    // Automute level functions
    /**
     * @brief Set automute activation level threshold
     * @param level Level threshold value
     */
    void setAutomuteLevel(uint16_t level) { writeRegister16(ES9039Q2M_AUTOMUTE_LEVEL, level); }
    
    /**
     * @brief Get automute activation level threshold
     * @return uint16_t Current level threshold
     */
    uint16_t getAutomuteLevel() { return readRegister16(ES9039Q2M_AUTOMUTE_LEVEL); }
    
    /**
     * @brief Set automute deactivation level threshold
     * @param level Level threshold value
     */
    void setAutomuteOffLevel(uint16_t level) { writeRegister16(ES9039Q2M_AUTOMUTE_OFF_LEVEL, level); }
    
    /**
     * @brief Get automute deactivation level threshold
     * @return uint16_t Current level threshold
     */
    uint16_t getAutomuteOffLevel() { return readRegister16(ES9039Q2M_AUTOMUTE_OFF_LEVEL); }

    // Automute soft ramp functions
    /**
     * @brief Set automute soft ramp rate
     * @param value Soft ramp rate value
     */
    void setAutomuteSoftRamp(uint8_t value) { writeRegisterMasked8(ES9039Q2M_AUTOMUTE_SOFT_RAMP, ES9039Q2M_BIT_AUTOMUTE_SOFT_RAMP_MASK, value); }
    
    /**
     * @brief Get automute soft ramp rate
     * @return uint8_t Current soft ramp rate value
     */
    uint8_t getAutomuteSoftRamp() { return readRegister8(ES9039Q2M_AUTOMUTE_SOFT_RAMP) & ES9039Q2M_BIT_AUTOMUTE_SOFT_RAMP_MASK; }

    // Program control bits
    /**
     * @brief Enable S/PDIF user bits loading
     */
    void enableSPDIFLoadUserBits() { writeRegisterMasked8(ES9039Q2M_PROGRAM_RAM_CONTROL, ES9039Q2M_BIT_SPDIF_LOAD_USER_BITS, ES9039Q2M_BIT_SPDIF_LOAD_USER_BITS); }
    
    /**
     * @brief Enable programmable coefficient write enable
     */
    void enableProgramCoeffWE() { writeRegisterMasked8(ES9039Q2M_PROGRAM_RAM_CONTROL, ES9039Q2M_BIT_PROGRAM_COEFF_WE, ES9039Q2M_BIT_PROGRAM_COEFF_WE); }
    
    /**
     * @brief Enable programmable coefficient enable
     */
    void enableProgramCoeffEN() { writeRegisterMasked8(ES9039Q2M_PROGRAM_RAM_CONTROL, ES9039Q2M_BIT_PROGRAM_COEFF_EN, ES9039Q2M_BIT_PROGRAM_COEFF_EN); }

    /**
     * @brief Get program RAM control register
     * @return uint8_t Current program RAM control value
     */
    uint8_t getProgramRAMControl() { return readRegister8(ES9039Q2M_PROGRAM_RAM_CONTROL); }

    // SPDIF read control functions
    /**
     * @brief Set SPDIF read control value
     * @param value SPDIF read control value
     */
    void setSPDIFReadControl(uint8_t value) { writeRegisterMasked8(ES9039Q2M_SPDIF_READ_CONTROL, ES9039Q2M_BIT_SPDIF_READ_CONTROL_MASK, value); }
    
    /**
     * @brief Get SPDIF read control value
     * @return uint8_t Current SPDIF read control value
     */
    uint8_t getSPDIFReadControl() { return readRegister8(ES9039Q2M_SPDIF_READ_CONTROL) & ES9039Q2M_BIT_SPDIF_READ_CONTROL_MASK; }

    // Program RAM address functions
    /**
     * @brief Set Program RAM address
     * @param address Program RAM address
     */
    void setProgramRAMAddress(uint8_t address) { writeRegister8(ES9039Q2M_PROGRAM_RAM_ADDRESS, address); }
    
    /**
     * @brief Get Program RAM address
     * @return uint8_t Current Program RAM address
     */
    uint8_t getProgramRAMAddress() { return readRegister8(ES9039Q2M_PROGRAM_RAM_ADDRESS); }

    /**
     * @brief Set Program RAM data
     * @param data Program RAM data
     */
    void setProgramRAMData(uint32_t data) { writeRegister24Signed(ES9039Q2M_PROGRAM_RAM_DATA, data); } // TODO: Handle 24 bit sign extension!
    
    /**
     * @brief Get Program RAM data
     * @return uint32_t Current Program RAM data
     */
    int32_t getProgramRAMData() { return readRegister24Signed(ES9039Q2M_PROGRAM_RAM_DATA); }  // TODO: Handle 24 bit sign extension!

    /**
     * @brief Get the chip ID
     * @return uint8_t Chip ID
     */
    uint8_t getChipID() { return readRegister8(ES9039Q2M_CHIP_ID); }

    /**
     * @brief Reads the interrupt states register
     * 
     * This function reads the 16-bit interrupt states register from the ES9039Q2M device
     * to determine which interrupt conditions are currently active.
     * 
     * @return uint16_t The current state of all interrupt flags as a 16-bit value
     * 
     * @note Each bit in the returned value corresponds to a specific interrupt condition
     * @see ES9039Q2M_INTERRUPT_STATES for the register address
     */
    uint16_t getInterruptStates() { return readRegister16(ES9039Q2M_INTERRUPT_STATES); } // TODO: Getters for individual bits?

    
    /**
     * @brief Reads the interrupt source register
     * 
     * This function reads the 16-bit interrupt source register from the ES9039Q2M device
     * to determine the source of the interrupt conditions.
     * 
     * @return uint16_t The current source of all interrupt flags as a 16-bit value
     * 
     * @note Each bit in the returned value corresponds to a specific interrupt source
     * @see ES9039Q2M_INTERRUPT_SOURCE for the register address
     */
    uint16_t getInterruptSource() { return readRegister16(ES9039Q2M_INTERRUPT_SOURCE); } // TODO: Getters for individual bits?

    /**
     * @brief Reads the current state of GPIO pins from the device.
     * 
     * This function retrieves the GPIO readback register value, which contains
     * the current digital state of all GPIO pins configured on the ES9039Q2M device.
     * 
     * @return uint8_t The 8-bit value representing the current GPIO pin states,
     *                 where each bit corresponds to an individual GPIO pin.
     */
    uint8_t getGPIOData() { return readRegister8(ES9039Q2M_GPIO_READBACK); }

    /**
     * @brief Get the minimum volume status 
     * @return uint8_t Current minimum volume status
     */
    uint8_t getVolMin() { return readRegister8(ES9039Q2M_GPIO_VOL_MIN_READ); }

    /**
     * @brief Get the minimum volume status of channel 1
     * @return uint8_t Current minimum volume status of channel 1
     */
    uint8_t getVolMinCh1() { return readRegister8(ES9039Q2M_GPIO_VOL_MIN_READ) & ES9039Q2M_BIT_MIN_VOL_CH1; }
    /**
     * @brief Get the minimum volume status of channel 2
     * @return uint8_t Current minimum volume status of channel 2
     */
    uint8_t getVolMinCh2() { return readRegister8(ES9039Q2M_GPIO_VOL_MIN_READ) & ES9039Q2M_BIT_MIN_VOL_CH2; }

    /**
     * @brief Get the auto mute status 
     * @return uint8_t Current auto mute status
     */
    uint8_t getAutoMute() { return readRegister8(ES9039Q2M_GPIO_AUTOMUTE_READ); }   
    /**
     * @brief Get the auto mute status of channel 1
     * @return uint8_t Current auto mute status of channel 1
     */
    uint8_t getAutoMuteCh1() { return readRegister8(ES9039Q2M_GPIO_AUTOMUTE_READ) & ES9039Q2M_BIT_AUTOMUTE_CH1; }
    /**
     * @brief Get the auto mute status of channel 2
     * @return uint8_t Current auto mute status of channel 2
     */ 
    uint8_t getAutoMuteCh2() { return readRegister8(ES9039Q2M_GPIO_AUTOMUTE_READ) & ES9039Q2M_BIT_AUTOMUTE_CH2; }

    // *********************************************
    // High level functions
    // *********************************************

    /**
     * @brief Set the volume level of both channels
     * @param volume Volume level (0.0 to 1.0 linear scale)
     */
    void setVolume(float volume);

    /**
     * @brief Set the volume level in decibels
     * @param volumeDB Volume level in dB (-127.5 to 0.0)
     */
    void setVolumeDB(float volumeDB);

    void setFilterParameters(uint8_t filterType, uint8_t filterValue); // TODO: Implement filter parameter setting

private:
    uint8_t _i2cAddress;
};
    
#endif // ES9039Q2M_H  