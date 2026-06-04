/**
 * STM32_EEPROM_I2C - Driver for 24C01 (and compatible 24Cxx) I2C EEPROM.
 *
 * Replaces the previous SPI EEPROM (M95010) driver. The 24C01 is a 1 Kbit
 * (128 byte) device with an 8-byte page write buffer, on I2C1 (PB6=SCL,
 * PB7=SDA). Write-protect is on PB5 (EE_WP).
 *
 * The public API intentionally mirrors the old SPI driver (eeRxBuffer,
 * EEPROM_StatusByte, *_ReadBuffer / *_WriteBuffer, EepromOperations) so the
 * application layer only needs minimal changes.
 */

#ifndef _STM32_EEPROM_I2C_H
#define _STM32_EEPROM_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 24C01 device parameters */
#define EEPROM_I2C_ADDR        0xA0  /*!< 8-bit I2C device address (A2/A1/A0 = GND) */
#define EEPROM_PAGESIZE        8     /*!< 24C01 page write buffer size (bytes)      */
#define EEPROM_MEM_SIZE        128   /*!< 24C01 total size (bytes)                  */

/* Buffer size used by the application (alarm data = 5 bytes, c/f data, ...) */
#define EEPROM_BUFFER_SIZE     16    /*!< EEPROM Buffer size. Setup to your needs */

/* EEPROM write-protect helpers (EE_WP high = protected, low = writable) */
#define EEPROM_WP_DISABLE()    HAL_GPIO_WritePin(EE_WP_GPIO_Port, EE_WP_Pin, GPIO_PIN_RESET)
#define EEPROM_WP_ENABLE()     HAL_GPIO_WritePin(EE_WP_GPIO_Port, EE_WP_Pin, GPIO_PIN_SET)

/**
 * @brief EEPROM Operations statuses
 */
typedef enum {
    EEPROM_STATUS_PENDING,
    EEPROM_STATUS_COMPLETE,
    EEPROM_STATUS_ERROR
} EepromOperations;

/* Application-visible buffers (kept for API compatibility with the old driver) */
extern uint8_t eeRxBuffer[EEPROM_BUFFER_SIZE];
extern uint8_t EEPROM_StatusByte;

void             EEPROM_I2C_INIT(I2C_HandleTypeDef * hi2c);
EepromOperations EEPROM_I2C_WriteBuffer(uint8_t * pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite);
EepromOperations EEPROM_I2C_ReadBuffer(uint8_t * pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead);
uint8_t          EEPROM_I2C_WaitStandbyState(void);

#ifdef __cplusplus
}
#endif

#endif // _STM32_EEPROM_I2C_H
