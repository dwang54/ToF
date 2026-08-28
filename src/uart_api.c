/**
 ******************************************************************************
 * @file    uart_api.c
 * @brief   Minimal blocking UART console on USART2.
 ******************************************************************************
 */

#include "uart_api.h"
#include "stm32f4xx_hal.h"

#include <stdio.h>
#include <string.h>

/* Change these three lines to move the console to a different USART. */
#define UART_INSTANCE      USART2
#define UART_GPIO_PORT     GPIOA
#define UART_GPIO_PINS     (GPIO_PIN_2 | GPIO_PIN_3)   /* PA2 = TX, PA3 = RX */
#define UART_GPIO_AF       GPIO_AF7_USART2

static UART_HandleTypeDef huart;
static bool               ready = false;

bool UART_API_Init(uint32_t baud)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    g.Pin       = UART_GPIO_PINS;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = UART_GPIO_AF;
    HAL_GPIO_Init(UART_GPIO_PORT, &g);

    huart.Instance          = UART_INSTANCE;
    huart.Init.BaudRate     = baud;
    huart.Init.WordLength   = UART_WORDLENGTH_8B;
    huart.Init.StopBits     = UART_STOPBITS_1;
    huart.Init.Parity       = UART_PARITY_NONE;
    huart.Init.Mode         = UART_MODE_TX_RX;
    huart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart) != HAL_OK) {
        ready = false;
        return false;
    }

    /* Unbuffered, so printf output appears immediately instead of sitting in
     * a newlib buffer waiting for a newline or a flush. */
    setvbuf(stdout, NULL, _IONBF, 0);

    ready = true;
    return true;
}

void UART_API_Print(const char *s)
{
    if (!ready || s == NULL) {
        return;
    }
    UART_API_Write((const uint8_t *)s, (uint16_t)strlen(s));
}

void UART_API_Write(const uint8_t *data, uint16_t len)
{
    if (!ready || data == NULL || len == 0U) {
        return;
    }
    HAL_UART_Transmit(&huart, (uint8_t *)data, len, HAL_MAX_DELAY);
}

bool UART_API_ReadByte(uint8_t *out)
{
    if (!ready || out == NULL) {
        return false;
    }
    /* 0 ms timeout -> returns immediately if nothing is waiting. */
    return (HAL_UART_Receive(&huart, out, 1, 0) == HAL_OK);
}

/* ------------------------------------------------------------------------- */
/* printf retarget. newlib calls _write(); we push the bytes out the UART.    */
/*                                                                           */
/* NOTE: delete the _write() currently in your main.c, or you get a           */
/* duplicate symbol at link time.                                            */
/* ------------------------------------------------------------------------- */
int _write(int file, char *ptr, int len)
{
    (void)file;
    UART_API_Write((const uint8_t *)ptr, (uint16_t)len);
    return len;
}