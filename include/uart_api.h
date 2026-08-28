/**
 ******************************************************************************
 * @file    uart_api.h
 * @brief   Minimal blocking UART console on USART2 (PA2/PA3 -> ST-LINK VCP).
 *
 *          Also retargets printf(), so after UART_API_Init() you can just
 *          use printf() normally.
 ******************************************************************************
 */

#ifndef UART_API_H
#define UART_API_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configure USART2 and its pins, and hook up printf().
 * @param baud  e.g. 115200. Use 921600 later when streaming sensor frames.
 * @return true on success.
 */
bool UART_API_Init(uint32_t baud);

/** @brief Send a NUL-terminated string. Blocking. */
void UART_API_Print(const char *s);

/** @brief Send raw bytes. Blocking. Use this for binary frames. */
void UART_API_Write(const uint8_t *data, uint16_t len);

/**
 * @brief Non-blocking single-byte read.
 * @param out  Receives the byte if one was waiting.
 * @return true if a byte was read, false if nothing available.
 */
bool UART_API_ReadByte(uint8_t *out);

#ifdef __cplusplus
}
#endif

#endif /* UART_API_H */