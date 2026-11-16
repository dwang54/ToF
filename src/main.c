/**
 ******************************************************************************
 * @file    main.c
 * @brief   Bringup for Nucleo-F446RE
 *
 *    
 ******************************************************************************
 */

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "gpio_api.h"
#include "uart_api.h"
#include "vl53l7cx_api.h"
#include "frame_encoder.h"


#define HYSTERESIS_MM  50      /* dead band; prevents flicker at the edge */
#define MIN_HITS      3       /* zones that must agree           */
#define SIGMA_MAX_MM  15
#define CENTER_ONLY   0       /* use only the middle 4x4         */

static bool led_on = false;
int16_t g_trigger_mm = 915;    /* <-- the distance you care about */

/* Set by EXTI4 ISR (GP_TOF_INT), consumed by the main loop. Replaces the
 * fixed HAL_Delay(5) poll: the sensor tells us exactly when a frame is
 * ready instead of us guessing. NOTE: PA4/EXTI4 mapping for GP_TOF_INT is
 * still UNVERIFIED per VL53L7CX_ROADMAP.md section 3 -- confirm against
 * the UM3067 schematic if this never fires. */
static volatile bool g_tof_int_flag = false;


I2C_HandleTypeDef hi2c1;                        /* Struct that holds info that HAL needs to manage one I2C peripheral instance*/
static VL53L7CX_Configuration dev;


/* 
    Initializes I2C1. Configures pin 8 and 9 as SCL and SDA respectively. 
*/
static bool I2C1_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();                   
    __HAL_RCC_I2C1_CLK_ENABLE();

    g.Pin       = GPIO_PIN_8 | GPIO_PIN_9;   /* PB8 = SCL (serial clk), PB9 = SDA (serial data) */
    g.Mode      = GPIO_MODE_AF_OD;           /* open drain */
    g.Pull      = GPIO_NOPULL;               /* board has its own pull-ups */
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &g);

    hi2c1.Instance             = I2C1;

    /* I2C configuration is done through init */
    hi2c1.Init.ClockSpeed      = 400000;                            // can we increase this to 1MHz? we can but it'd require enabling a macro and calculating CCR and TRISE ourself
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    return (HAL_I2C_Init(&hi2c1) == HAL_OK);
}



/*
    MCU drives serial clk and serial data goes low - devices on bus start noticing. We receive ACKS from all devices on the bus that are listening.
*/
static void I2C1_Scan(void)
{
    printf("Scanning I2C1...\r\n");
    int n = 0;
    for (uint8_t a = 1; a < 128; a++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 2, 10) == HAL_OK) {
            printf("  ACK: 7-bit 0x%02X  (8-bit 0x%02X)\r\n", a, a << 1);
            n++;
        }
    }
    printf("%d device(s)\r\n", n);
}



/* GP_TOF_INT lives on PA4 (see gpio_api.c). Re-init that one pin in
 * interrupt mode -- GPIO_API_Init() already set it up as a plain
 * pulled-up input, this layers EXTI on top without touching anything else. */
static void TOF_INT_EXTI_Init(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_4;
    g.Mode  = GPIO_MODE_IT_FALLING;   /* INT is open-drain, active low, pulled up */
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);

    HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

/* Weak default lives in stm32f4xx_hal_gpio.c; this override is what actually
 * runs. Keep it tiny -- it executes at interrupt priority 2. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_4) {
        g_tof_int_flag = true;
    }
}

/* The vector table's default handler for this line is a weak infinite loop
 * (no stm32f4xx_it.c exists in this project) -- this definition is what
 * gets linked instead. */
void EXTI4_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

static void proximity_update(const VL53L7CX_ResultsData *res)
{
    /* Hysteresis: the bar to STAY on is lower than the bar to turn on. */
    int16_t threshold = led_on ? (g_trigger_mm + HYSTERESIS_MM) : g_trigger_mm;

    int hits = 0;
    int16_t nearest = 32767;

    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
        #if CENTER_ONLY
            if (r < 2 || r > 5 || c < 2 || c > 5) continue;
        #endif
            int i = r * 8 + c;

            uint8_t st = res->target_status[i];
            if (st != 5 && st != 9)               continue;
            if (res->range_sigma_mm[i] > SIGMA_MAX_MM) continue;

            int16_t d = res->distance_mm[i];
            if (d < nearest) nearest = d;
            if (d < threshold) hits++;
        }
    }

    // bool was_on = led_on;
    led_on = (hits >= MIN_HITS);
    GPIO_API_Write(GP_LED, led_on);

    // if (led_on != was_on) {
    //     printf("%s  nearest %d mm  hits %d\r\n",
    //            led_on ? "ON " : "off", nearest, hits);
    // }
    
}




int main(void)
{

    HAL_Init();                                                         /* Initialize HAL library */
    GPIO_API_Init();                                                    /* Init GPIO */
    // GPIO_API_Write(GP_LED, true);

    if (!UART_API_Init(115200)) {                                       /* Init UART */
        for (;;) {
            GPIO_API_Toggle(GP_LED);
            for (volatile uint32_t d = 0; d < 100000; d++) { __NOP(); }
        }
    }

    HAL_Delay(1000);
    printf("\r\n--- UART OK ---\r\n");
    printf("SYSCLK : %lu Hz\r\n", (unsigned long)HAL_RCC_GetSysClockFreq());
    printf("PCLK1  : %lu Hz\r\n", (unsigned long)HAL_RCC_GetPCLK1Freq());
    printf("encoder selftest: %s\r\n", frame_encoder_selftest() ? "PASS" : "FAIL");

    GPIO_API_Write(GP_TOF_PWR_EN, true);   HAL_Delay(10);   /* enables sensor's supply rail*/
    GPIO_API_Write(GP_TOF_LPN,    true);   HAL_Delay(10);   /* LPN is comms enable, 1 = comms enabled */

                      

    // clears I2C interface, resets state machine to a known state. I2C_RST is active high
    GPIO_API_Write(GP_TOF_I2C_RST, true);  HAL_Delay(1);
    GPIO_API_Write(GP_TOF_I2C_RST, false); HAL_Delay(1);

    if (!I2C1_Init()) { printf("I2C init failed\r\n"); }                /* Init I2C */
    I2C1_Scan();

    memset(&dev, 0, sizeof(dev));
    dev.platform.address = VL53L7CX_DEFAULT_I2C_ADDRESS;   /* 0x52, address of device to talk to */   

    uint8_t alive = 0;
    uint8_t st = vl53l7cx_is_alive(&dev, &alive);
    printf("is_alive: status=%u alive=%u\r\n", st, alive);


    printf("init (uploads 84 KB, ~2 s at 400 kHz)...\r\n");

    uint32_t t0 = HAL_GetTick();
    st = vl53l7cx_init(&dev);
    printf("init: status=%u  took %lu ms\r\n", st, (unsigned long)(HAL_GetTick() - t0));
    printf("trigger at %d mm (off at %d)\r\n", g_trigger_mm, g_trigger_mm + HYSTERESIS_MM);

    /*
        Configuring sensor 
    */
    // sensor default boots in 4x4, we change it to 8x8 for 64 zones
    vl53l7cx_set_resolution(&dev, VL53L7CX_RESOLUTION_8X8);   /* before frequency */
    vl53l7cx_set_target_order(&dev, VL53L7CX_TARGET_ORDER_CLOSEST);
    vl53l7cx_set_ranging_frequency_hz(&dev, 15);

    /* Explicit integration time. Must stay below the ranging period
     * (1000/freq_hz ms) -- at 15 Hz that's a 66 ms budget, so 20 ms leaves
     * comfortable headroom. Longer integration = more collected photons =
     * better SNR at range, at the cost of motion blur on fast-moving
     * targets. Bump this toward the ~60 ms ceiling for max-range bench
     * tests on a static target; drop it back down for the pendulum/
     * reaction-time test where you want the sharpest possible timing. */
    st = vl53l7cx_set_integration_time_ms(&dev, 20);
    printf("set_integration_time_ms(20): status=%u\r\n", st);

    uint32_t itime = 0;
    vl53l7cx_get_integration_time_ms(&dev, &itime);
    printf("integration time readback: %lu ms\r\n", (unsigned long)itime);

    vl53l7cx_start_ranging(&dev);
    TOF_INT_EXTI_Init();   /* after start_ranging: sensor now drives INT low on new data */

    static VL53L7CX_ResultsData res;
    uint8_t ready = 0;


    /* Interrupt-driven: GP_TOF_INT (PA4) pulses low when a frame is ready,
     * EXTI4_IRQHandler -> HAL_GPIO_EXTI_Callback sets g_tof_int_flag, and
     * the loop below just drains it. No fixed HAL_Delay poll interval
     * means no added latency/jitter between "frame ready" and "frame read" --
     * useful for the reaction-time / motion test. */

        uint32_t seq = 0;

        for (;;) {
            if (g_tof_int_flag) {
                g_tof_int_flag = false;
                vl53l7cx_check_data_ready(&dev, &ready);
                if (ready) {
                    vl53l7cx_get_ranging_data(&dev, &res);
                    /* _ext variant adds signal_per_spad + ambient_per_spad
                     * per zone, for range/SNR characterization logging
                     * (see tools/csv_logger.py). Switch back to plain
                     * frame_send_ranging() if you need the higher frame
                     * rate headroom instead. */
                    frame_send_ranging_ext(&res, 8, seq++);
                    proximity_update(&res);
                }
            }
            __WFI();   /* sleep until the next interrupt (EXTI4 or SysTick) */
        }


}



/* The framework's startup file only provides a weak SysTick_Handler that
 * spins forever. Nothing here calls HAL_Delay(), so this is not strictly
 * required yet -- but leave it in so the first HAL_Delay() you add later
 * does not hang the MCU. */
void SysTick_Handler(void)
{
    HAL_IncTick();
}