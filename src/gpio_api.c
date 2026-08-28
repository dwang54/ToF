/**
 ******************************************************************************
 * @file    gpio_api.c
 * @brief   Named-pin GPIO layer. All physical mapping lives in gp_map[].
 ******************************************************************************
 */

#include "gpio_api.h"
#include "stm32f4xx_hal.h"

/* ------------------------------------------------------------------------- */
/* Physical mapping                                                          */
/*                                                                           */
/* GP_LED is confirmed: LD2 is PA5 on every Nucleo-64.                       */
/*                                                                           */
/* !! THE FOUR GP_TOF_* ENTRIES BELOW ARE PLACEHOLDERS. !!                   */
/* ST's CubeMX example for this board configures PB0, PB3, PB4 as outputs    */
/* and PA4 as EXTI4 for the PWR_EN / LPn / I2C_RST / INT group -- but which  */
/* signal lands on which of PB0/PB3/PB4 needs confirming against the UM3067  */
/* schematic before you trust these. Getting LPn wrong means the sensor      */
/* never ACKs and you will blame your I2C code.                              */
/* ------------------------------------------------------------------------- */

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    gp_mode_t     mode;
    bool          reset_level;   /* level applied at Init, outputs only */
} gp_map_entry_t;

static gp_map_entry_t gp_map[GP_PIN_COUNT] =
{
    /* [GP_LED]          */ { GPIOA, GPIO_PIN_5, GP_MODE_OUTPUT, false },

    /* [GP_TOF_PWR_EN]   */ { GPIOB, GPIO_PIN_0, GP_MODE_OUTPUT, false },  /* VERIFY */
    /* [GP_TOF_LPN]      */ { GPIOB, GPIO_PIN_4, GP_MODE_OUTPUT, false },  /* VERIFY */
    /* [GP_TOF_I2C_RST]  */ { GPIOB, GPIO_PIN_3, GP_MODE_OUTPUT, false },  /* VERIFY */
    /* [GP_TOF_INT]      */ { GPIOA, GPIO_PIN_4, GP_MODE_INPUT_PU, false },/* VERIFY */
};

/* ------------------------------------------------------------------------- */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------- */

static void gp_enable_port_clock(const GPIO_TypeDef *port)
{
    if      (port == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); }
    else if (port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
    else if (port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
    else if (port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
    else if (port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
    else if (port == GPIOH) { __HAL_RCC_GPIOH_CLK_ENABLE(); }
    /* F446RE in LQFP64 only bonds out A, B, C, D2, H0, H1. */
}

static void gp_apply_mode(const gp_map_entry_t *e)
{
    GPIO_InitTypeDef g = {0};

    g.Pin   = e->pin;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    switch (e->mode) {
    case GP_MODE_OUTPUT:
        g.Mode = GPIO_MODE_OUTPUT_PP;
        g.Pull = GPIO_NOPULL;
        break;
    case GP_MODE_INPUT_PU:
        g.Mode = GPIO_MODE_INPUT;
        g.Pull = GPIO_PULLUP;
        break;
    case GP_MODE_INPUT_PD:
        g.Mode = GPIO_MODE_INPUT;
        g.Pull = GPIO_PULLDOWN;
        break;
    case GP_MODE_INPUT:
    default:
        g.Mode = GPIO_MODE_INPUT;
        g.Pull = GPIO_NOPULL;
        break;
    }

    HAL_GPIO_Init(e->port, &g);
}

static bool gp_valid(gp_pin_t pin)
{
    return ((int)pin >= 0) && (pin < GP_PIN_COUNT) && (gp_map[pin].port != NULL);
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

void GPIO_API_Init(void)
{
    for (int i = 0; i < GP_PIN_COUNT; i++) {
        gp_map_entry_t *e = &gp_map[i];

        if (e->port == NULL) {
            continue;
        }

        gp_enable_port_clock(e->port);

        /* Set the output level *before* switching to output so the pin never
         * momentarily drives the wrong way. Matters for PWR_EN and LPn. */
        if (e->mode == GP_MODE_OUTPUT) {
            HAL_GPIO_WritePin(e->port, e->pin,
                              e->reset_level ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        gp_apply_mode(e);
    }
}

void GPIO_API_Write(gp_pin_t pin, bool level)
{
    if (!gp_valid(pin)) {
        return;
    }
    HAL_GPIO_WritePin(gp_map[pin].port, gp_map[pin].pin,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool GPIO_API_Read(gp_pin_t pin)
{
    if (!gp_valid(pin)) {
        return false;
    }
    return (HAL_GPIO_ReadPin(gp_map[pin].port, gp_map[pin].pin) == GPIO_PIN_SET);
}

void GPIO_API_Toggle(gp_pin_t pin)
{
    if (!gp_valid(pin)) {
        return;
    }
    HAL_GPIO_TogglePin(gp_map[pin].port, gp_map[pin].pin);
}

void GPIO_API_SetMode(gp_pin_t pin, gp_mode_t mode)
{
    if (!gp_valid(pin)) {
        return;
    }
    gp_map[pin].mode = mode;
    gp_apply_mode(&gp_map[pin]);
}