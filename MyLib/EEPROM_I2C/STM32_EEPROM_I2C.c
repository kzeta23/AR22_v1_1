/**
 * STM32_EEPROM_I2C - Driver for 24C01 (and compatible 24Cxx) I2C EEPROM.
 *
 * Uses the STM32 HAL I2C memory API. Writes are split on the device's
 * page boundary (EEPROM_PAGESIZE) and each page write is followed by ACK
 * polling so the caller may pass an arbitrary length / address.
 */

#include "STM32_EEPROM_I2C.h"

I2C_HandleTypeDef * EEPROM_I2C;
uint8_t EEPROM_StatusByte;
uint8_t eeRxBuffer[EEPROM_BUFFER_SIZE] = {0x00};

#define EEPROM_I2C_TIMEOUT      200U   /* per-transfer timeout (ms)        */
#define EEPROM_I2C_TRIALS       300U   /* ACK-poll trials after a write    */

/**
 * @brief  Store the I2C handle used to talk to the EEPROM.
 * @param  hi2c Pointer to the initialised I2C handle (e.g. &hi2c1)
 */
void EEPROM_I2C_INIT(I2C_HandleTypeDef * hi2c) {
    EEPROM_I2C = hi2c;
}

/**
 * @brief  Wait until the EEPROM has finished its internal write cycle by
 *         polling for an ACK to its address.
 * @retval 0 on success, 1 on timeout/error
 */
uint8_t EEPROM_I2C_WaitStandbyState(void) {
    if (HAL_I2C_IsDeviceReady(EEPROM_I2C, EEPROM_I2C_ADDR, EEPROM_I2C_TRIALS,
                              EEPROM_I2C_TIMEOUT) != HAL_OK) {
        return 1;
    }
    return 0;
}

/**
 * @brief  Write a single page (max EEPROM_PAGESIZE bytes, no page-boundary
 *         crossing) to the EEPROM.
 */
static EepromOperations EEPROM_I2C_WritePage(uint8_t * pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite) {
    HAL_StatusTypeDef status;

    EEPROM_WP_DISABLE();

    status = HAL_I2C_Mem_Write(EEPROM_I2C, EEPROM_I2C_ADDR, WriteAddr,
                               I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToWrite,
                               EEPROM_I2C_TIMEOUT);

    /* Wait for the internal write cycle to complete (tWR, typ. 5 ms) */
    EEPROM_I2C_WaitStandbyState();

    EEPROM_WP_ENABLE();

    return (status == HAL_OK) ? EEPROM_STATUS_COMPLETE : EEPROM_STATUS_ERROR;
}

/**
 * @brief  Write a block of data, splitting it across page boundaries.
 * @param  pBuffer        data to write
 * @param  WriteAddr      EEPROM start address
 * @param  NumByteToWrite number of bytes
 * @retval EEPROM_STATUS_COMPLETE or EEPROM_STATUS_ERROR
 */
EepromOperations EEPROM_I2C_WriteBuffer(uint8_t * pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite) {
    EepromOperations status = EEPROM_STATUS_COMPLETE;

    while (NumByteToWrite > 0) {
        /* Bytes left in the current page */
        uint16_t pageRemain = EEPROM_PAGESIZE - (WriteAddr % EEPROM_PAGESIZE);
        uint16_t chunk = (NumByteToWrite < pageRemain) ? NumByteToWrite : pageRemain;

        status = EEPROM_I2C_WritePage(pBuffer, WriteAddr, chunk);
        if (status != EEPROM_STATUS_COMPLETE) {
            return status;
        }

        WriteAddr      += chunk;
        pBuffer        += chunk;
        NumByteToWrite -= chunk;
    }

    return status;
}

/**
 * @brief  Read a block of data. Sequential reads may cross pages freely.
 * @param  pBuffer       destination buffer
 * @param  ReadAddr      EEPROM start address
 * @param  NumByteToRead number of bytes
 * @retval EEPROM_STATUS_COMPLETE or EEPROM_STATUS_ERROR
 */
EepromOperations EEPROM_I2C_ReadBuffer(uint8_t * pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead) {
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(EEPROM_I2C, EEPROM_I2C_ADDR, ReadAddr,
                              I2C_MEMADD_SIZE_8BIT, pBuffer, NumByteToRead,
                              EEPROM_I2C_TIMEOUT);

    return (status == HAL_OK) ? EEPROM_STATUS_COMPLETE : EEPROM_STATUS_ERROR;
}
