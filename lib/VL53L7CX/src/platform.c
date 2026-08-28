/**
 ******************************************************************************
 * @file    platform.c
 * @brief   VL53L7CX ULD platform layer for STM32 Nucleo-F446RE (I2C1)
 ******************************************************************************
 */

#include "platform.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* The I2C peripheral handle. Defined in main.c by MX_I2C1_Init().            */
extern I2C_HandleTypeDef hi2c1;

/* HAL's Size argument is 16-bit, but the firmware upload (WrMulti) can be     */
/* larger (~84 KB). We chunk large writes; the sensor auto-increments its      */
/* internal register pointer, so we advance RegisterAdress by the chunk size.  */
#define VL53L7CX_COMMS_CHUNK_SIZE   4096U
#define VL53L7CX_I2C_TIMEOUT_MS     1000U

/* ------------------------------------------------------------------------- */
/* Read one byte                                                             */
/* ------------------------------------------------------------------------- */
uint8_t VL53L7CX_RdByte(
        VL53L7CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_value)
{
    return VL53L7CX_RdMulti(p_platform, RegisterAdress, p_value, 1);
}

/* ------------------------------------------------------------------------- */
/* Write one byte                                                            */
/* ------------------------------------------------------------------------- */
uint8_t VL53L7CX_WrByte(
        VL53L7CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t value)
{
    return VL53L7CX_WrMulti(p_platform, RegisterAdress, &value, 1);
}

/* ------------------------------------------------------------------------- */
/* Read multiple bytes                                                       */
/* ------------------------------------------------------------------------- */
uint8_t VL53L7CX_RdMulti(
        VL53L7CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_values,
        uint32_t size)
{
    uint32_t i = 0;

    while (i < size) {
        uint16_t chunk = (size - i > VL53L7CX_COMMS_CHUNK_SIZE)
                       ? VL53L7CX_COMMS_CHUNK_SIZE
                       : (uint16_t)(size - i);

        if (HAL_I2C_Mem_Read(&hi2c1,
                             p_platform->address,       /* 8-bit addr (0x52) */
                             (uint16_t)(RegisterAdress + i),
                             I2C_MEMADD_SIZE_16BIT,
                             &p_values[i],
                             chunk,
                             VL53L7CX_I2C_TIMEOUT_MS) != HAL_OK) {
            return 255;                                  /* non-zero = error  */
        }
        i += chunk;
    }
    return 0;                                            /* 0 = OK            */
}

/* ------------------------------------------------------------------------- */
/* Write multiple bytes (also used for the ~84 KB firmware upload)           */
/* ------------------------------------------------------------------------- */
uint8_t VL53L7CX_WrMulti(
        VL53L7CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_values,
        uint32_t size)
{
    uint32_t i = 0;

    while (i < size) {
        uint16_t chunk = (size - i > VL53L7CX_COMMS_CHUNK_SIZE)
                       ? VL53L7CX_COMMS_CHUNK_SIZE
                       : (uint16_t)(size - i);

        if (HAL_I2C_Mem_Write(&hi2c1,
                              p_platform->address,
                              (uint16_t)(RegisterAdress + i),
                              I2C_MEMADD_SIZE_16BIT,
                              &p_values[i],
                              chunk,
                              VL53L7CX_I2C_TIMEOUT_MS) != HAL_OK) {
            return 255;
        }
        i += chunk;
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/* Swap a buffer of 32-bit words (size is always a multiple of 4)            */
/* Called by the driver regardless of endianness.                            */
/* ------------------------------------------------------------------------- */
void VL53L7CX_SwapBuffer(
        uint8_t *buffer,
        uint16_t size)
{
    uint32_t i, tmp;

    for (i = 0; i < size; i += 4) {
        tmp = ((uint32_t)buffer[i]     << 24)
            | ((uint32_t)buffer[i + 1] << 16)
            | ((uint32_t)buffer[i + 2] <<  8)
            |  (uint32_t)buffer[i + 3];

        memcpy(&buffer[i], &tmp, 4);
    }
}

/* ------------------------------------------------------------------------- */
/* Blocking wait                                                             */
/* ------------------------------------------------------------------------- */
uint8_t VL53L7CX_WaitMs(
        VL53L7CX_Platform *p_platform,
        uint32_t TimeMs)
{
    (void)p_platform;
    HAL_Delay(TimeMs);
    return 0;
}