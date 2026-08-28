/**
 ******************************************************************************
 * @file    gpio_api.h
 * @brief   Thin named-pin GPIO layer for the Nucleo-F446RE + X-NUCLEO-53L7A1.
 *
 *          Call sites refer to logical pins (GP_LED, GP_TOF_LPN, ...) instead
 *          of raw port/pin pairs. The physical mapping lives in one table in
 *          gpio_api.c, so re-wiring is a single-line edit.
 ******************************************************************************
 */

#ifndef GPIO_API_H
#define GPIO_API_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Logical pin identifiers.
 *
 * NOTE: deliberately prefixed GP_ rather than GPIO_PIN_ -- the HAL already
 * owns GPIO_PIN_0 .. GPIO_PIN_15 and colliding with those is a nasty,
 * hard-to-read compile error.
 *
 * GP_PIN_COUNT must stay last; it sizes the mapping table.
 */
typedef enum
{
    GP_LED = 0,          /* LD2, PA5, active high                            */

    GP_TOF_PWR_EN,       /* sensor supply enable                             */
    GP_TOF_LPN,          /* low-power / comms enable -- HIGH to talk to it   */
    GP_TOF_I2C_RST,      /* sensor I2C reset                                 */
    GP_TOF_INT,          /* ranging-data-ready, input                        */

    GP_PIN_COUNT
} gp_pin_t;

/** @brief Pin direction / termination. */
typedef enum
{
    GP_MODE_OUTPUT = 0,  /* push-pull output      */
    GP_MODE_INPUT,       /* floating input        */
    GP_MODE_INPUT_PU,    /* input, pull-up        */
    GP_MODE_INPUT_PD     /* input, pull-down      */
} gp_mode_t;

/**
 * @brief Enable the required port clocks and configure every pin in the table.
 *        Safe to call more than once. Outputs are driven to their configured
 *        reset level before the pin is switched to output, so nothing glitches.
 */
void GPIO_API_Init(void);

/**
 * @brief Drive an output pin.
 * @param pin   Logical pin. Out-of-range values are ignored.
 * @param level true = high, false = low.
 */
void GPIO_API_Write(gp_pin_t pin, bool level);

/**
 * @brief Read the current level of a pin (input or output).
 * @return true if high. Returns false for an out-of-range pin.
 */
bool GPIO_API_Read(gp_pin_t pin);

/** @brief Invert an output pin. */
void GPIO_API_Toggle(gp_pin_t pin);

/**
 * @brief Reconfigure a pin's direction at runtime.
 *        Useful for the ToF INT line if you later switch it to EXTI.
 */
void GPIO_API_SetMode(gp_pin_t pin, gp_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_API_H */