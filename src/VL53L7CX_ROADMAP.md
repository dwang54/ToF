# VL53L7CX Bring-Up Roadmap
### Nucleo-F446RE + X-NUCLEO-53L7A1 + SATEL-VL53L7CX

---

## 0. Where you actually are

| Item | State |
|---|---|
| Toolchain, PlatformIO, HAL | Working |
| Firmware upload | Working (confirmed by behavior changes tracking code changes) |
| LD2 / GPIO | Working |
| `SysTick` / `HAL_Delay` | Working |
| USART2 → ST-LINK VCP | **Works only with the expansion board removed** |
| HSE bypass (8 MHz from ST-LINK MCO) | **Broken — do not use** |
| ST-LINK ↔ OpenOCD (`pio debug`) | **Broken — `LIBUSB_ERROR_NOT_FOUND`** |
| 53L7A1 control pin mapping | **Unverified** |
| I²C1 | Not started |
| VL53L7CX driver | Compiles clean; never executed |

Four open problems. Three of them (the VCP conflict, HSE, OpenOCD) must be
resolved or worked around *before* sensor bring-up, because without a debug
channel you will be flying blind through the trickiest part of the project.

---

## 1. Unblock your debug channel — do this first

You cannot bring up an I²C sensor with no way to print. This is the single
highest-priority item.

### 1a. Diagnose the PA2/PA3 conflict

**Symptom (confirmed):** expansion board off → serial output. Expansion board
on → total silence.
**Mechanism (not confirmed):** most likely the 53L7A1 drives or loads PA2/PA3,
which appear on the Arduino header as D1/D0.

Rule out the alternatives before you accept the diagnosis:

1. **Power brownout.** With the board mounted, is LD3 still solid? A sagging
   3.3 V rail causes reset loops that look identical to a dead UART.
2. **Reset loop.** Add a boot counter in a `__attribute__((section(".noinit")))`
   variable, or simply watch whether LD2 restarts its pattern.
3. **Direct measurement.** Probe PA2 (CN9 pin 1 / D1) with a scope or logic
   analyzer while the heartbeat runs, board mounted. If you see UART activity
   on the pin but nothing at the PC, the break is between PA2 and the ST-LINK.
   If the pin is held low or flat, the expansion board is driving it.
4. **Read the schematic.** UM3067 has the 53L7A1 schematic. Find every Arduino
   header pin the board connects to and check whether D0/D1 are among them.

### 1b. Pick a workaround

| Option | Cost | Notes |
|---|---|---|
| **Move console to USART1 (PA9/PA10)** | Needs a $3 USB-UART adapter | Cleanest. PA9/PA10 are D8/D2 on the header — verify they are free on the 53L7A1 first. |
| **Move console to USART6 (PC6/PC7)** | Same adapter | Another candidate if PA9/PA10 clash. |
| **Cut/reroute the solder bridges** | Free, irreversible-ish | Nucleo-64 routes PA2/PA3 to *either* the VCP *or* the Arduino header via solder bridges. Check UM1724 for your board revision before touching anything — I am not confident of the bridge numbers and a wrong cut is annoying to undo. |
| **SWO / ITM trace** | Free | Requires working OpenOCD, which you do not have yet. Revisit after §1c. |
| **Blink codes only** | Free | Viable for coarse status; useless for 64-zone distance data. |

**Recommendation:** buy a cheap CP2102/CH340 USB-UART adapter and move the
console to USART1. It costs a few dollars, removes the ST-LINK VCP from your
dependency chain entirely, and lets you run at 921600 baud later when you are
streaming 8×8 frames — which the VCP would struggle with anyway.

### 1c. Fix the ST-LINK (fixes two problems at once)

`LIBUSB_ERROR_NOT_FOUND` and the dead HSE bypass share a likely root cause:
**stale ST-LINK firmware.** The MCO output that feeds HSE comes from the
ST-LINK MCU; old firmware can fail to produce it, and old firmware is also the
most common cause of OpenOCD probe failures.

1. Run **STSW-LINK007** (`ST-LinkUpgrade.exe`) → *Device Connect* → *Yes*.
2. If OpenOCD still fails, install **STSW-LINK009** (USB driver) and reboot.
3. If you have ever run Zadig against this device, revert it in Device Manager
   under *STMicroelectronics STLink dongle*.

Recovering OpenOCD gets you `pio debug` — single-stepping through
`vl53l7cx_init()` is worth a great deal when the firmware upload fails.

---

## 2. Settle the clock

**Do not go back to `RCC_HSE_BYPASS`** until §1c is done and you have verified
the MCO with a scope.

### Option A — stay on 16 MHz HSI (recommended for bring-up)

Delete `SystemClock_Config()` entirely. `SystemInit()` already leaves you on
HSI at 16 MHz with `SystemCoreClock` correct.

This is genuinely sufficient:
- I²C Fast-mode (400 kHz) requires PCLK1 ≥ 4 MHz. You have 16 MHz.
- The 84 KB firmware upload is I²C-bound, not CPU-bound: ~1.9 s at 400 kHz
  regardless of core speed.
- The ULD does no heavy computation; it is a transport-and-unpack driver.

### Option B — 84 MHz from the HSI PLL

When you want the headroom (streaming, filtering, multiple sensors):

```c
osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
osc.HSIState            = RCC_HSI_ON;
osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
osc.PLL.PLLState        = RCC_PLL_ON;
osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
osc.PLL.PLLM            = 8;              /* 16 / 8   = 2 MHz   */
osc.PLL.PLLN            = 168;            /* 2 * 168  = 336 MHz */
osc.PLL.PLLP            = RCC_PLLP_DIV4;  /* 336 / 4  = 84 MHz  */
osc.PLL.PLLQ            = 7;
osc.PLL.PLLR            = 2;
```

Keep `FLASH_LATENCY_2`, `APB1 = HCLK/2`, `APB2 = HCLK/1`.

HSI is ±1 % over temperature vs a crystal's ±0.5 %. Irrelevant for I²C (the
sensor is a slave and follows your clock) and fine for UART up to ~460800.

**Verification, whichever you choose:** print `HAL_RCC_GetSysClockFreq()` and
`HAL_RCC_GetPCLK1Freq()` at boot, and independently confirm with
`HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_4)` on PA8 plus
a scope. Register math tells you what the MCU *believes*; only a measurement
tells you what is *true*. That distinction is exactly what cost you the last
several hours.

---

## 3. Verify the 53L7A1 control pins

The `GP_TOF_*` entries in `gpio_api.c` are **guesses**. Confirm against the
UM3067 schematic before writing any I²C code.

You need four signals for the centre sensor:

| Signal | Direction | Purpose | Consequence if wrong |
|---|---|---|---|
| `PWR_EN` | out | Sensor supply enable | Sensor unpowered → no ACK |
| `LPn` | out | Comms enable — **must be HIGH to talk I²C** | Sensor ignores you entirely |
| `I2C_RST` | out | I²C block reset | Sensor stuck after a bad transaction |
| `INT` | in | Data-ready | Polling still works; you just lose the interrupt |

ST's CubeMX example for this board configures **PB0, PB3, PB4** as outputs and
**PA4** as EXTI4 for this group — but which signal lands on which of PB0/PB3/PB4
is what you must confirm.

**Empirical fallback if the schematic is unavailable:** write a loop that tries
all six permutations of PB0/PB3/PB4 → {PWR_EN, LPn, I2C_RST}, driving all three
high, and after each permutation run an I²C scan. The one that produces an ACK
at 0x29 is correct. Crude, but it takes two minutes and is definitive.

---

## 4. I²C1 bring-up

### Wiring
- **SCL = PB8**, **SDA = PB9** (Arduino D15/D14), `GPIO_AF4_I2C1`
- `GPIO_MODE_AF_OD` (open drain), `GPIO_NOPULL` — the expansion board has its
  own pull-ups. Do not add internal ones.

### Init

```c
__HAL_RCC_GPIOB_CLK_ENABLE();
__HAL_RCC_I2C1_CLK_ENABLE();

GPIO_InitTypeDef g = {0};
g.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
g.Mode      = GPIO_MODE_AF_OD;
g.Pull      = GPIO_NOPULL;
g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
g.Alternate = GPIO_AF4_I2C1;
HAL_GPIO_Init(GPIOB, &g);

hi2c1.Instance             = I2C1;
hi2c1.Init.ClockSpeed      = 400000;              /* Fast mode */
hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
hi2c1.Init.OwnAddress1     = 0;
hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Fault(5); }
```

**400 kHz, not 100 kHz.** The firmware upload is 84 KB. At 100 kHz that is
~7.7 s of blocking I²C every boot; at 400 kHz it is ~1.9 s. ST explicitly
recommends 400 kHz for this sensor.

### Bus scan (write this before touching the driver)

```c
for (uint8_t a = 1; a < 128; a++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, a << 1, 2, 10) == HAL_OK) {
        printf("ACK at 7-bit 0x%02X (8-bit 0x%02X)\r\n", a, a << 1);
    }
}
```

**Expect an ACK at 7-bit `0x29`.** The ULD and your `platform.c` use the 8-bit
form `0x52` — same device, shifted left one bit. This trips up nearly everyone;
`VL53L7CX_DEFAULT_I2C_ADDRESS` is already `0x52`, so just do not "fix" it.

If the scan finds nothing: LPn is low, PWR_EN is low, the pin mapping is wrong,
or SCL/SDA are swapped. It is not your driver.

### Power-up sequence

```
PWR_EN high  →  wait 10 ms
LPn     high  →  wait 10 ms
I2C_RST high  →  wait 1 ms  →  I2C_RST low  →  wait 1 ms
                              (pulse resets the sensor's I2C block)
scan / is_alive
```

---

## 5. Driver bring-up

### 5a. Prove life

```c
static VL53L7CX_Configuration dev = {0};   /* 2776 bytes — keep it static */
uint8_t alive = 0, status;

dev.platform.address = VL53L7CX_DEFAULT_I2C_ADDRESS;   /* 0x52 */

status = vl53l7cx_is_alive(&dev, &alive);
printf("is_alive: status=%u alive=%u\r\n", status, alive);
```

`vl53l7cx_is_alive` reads device ID (expect `0xF0`) and revision (expect `0x02`).
If this fails, stop — do not attempt `init`.

### 5b. Initialise

```c
status = vl53l7cx_init(&dev);     /* uploads 84 KB of firmware, ~2 s */
printf("init: status=%u\r\n", status);
```

This is the single most failure-prone call in the project. It does three
`WrMulti` transfers of 0x8000/0x8000/0x5000 bytes, chunked at 4096 by your
`platform.c`. Things that break it:

- **I²C timeout too short.** A 4096-byte chunk at 400 kHz is ~100 ms; your
  `VL53L7CX_I2C_TIMEOUT_MS` of 1000 gives 10× margin. At 100 kHz it is ~410 ms
  — only 2.4× margin. Another reason for 400 kHz.
- **`SysTick` not running.** `vl53l7cx_init` calls `VL53L7CX_WaitMs()` many
  times; without the handler it hangs forever. (See Appendix A.)
- **Noisy bus.** Long jumper wires to a satellite breakout will corrupt an
  84 KB transfer. Keep leads short.

Print the status after *every* driver call during bring-up. The ULD returns
0 for OK and accumulates errors with `|=`, so a non-zero value tells you
something failed but not where — granularity has to come from you.

### 5c. Configure and range

```c
vl53l7cx_set_resolution(&dev, VL53L7CX_RESOLUTION_8X8);
vl53l7cx_set_ranging_frequency_hz(&dev, 10);
vl53l7cx_set_ranging_mode(&dev, VL53L7CX_RANGING_MODE_CONTINUOUS);
vl53l7cx_start_ranging(&dev);

VL53L7CX_ResultsData results;
uint8_t ready = 0;

for (;;) {
    vl53l7cx_check_data_ready(&dev, &ready);
    if (ready) {
        vl53l7cx_get_ranging_data(&dev, &results);
        /* ... consume ... */
    }
    HAL_Delay(5);
}
```

**Order matters:** `set_resolution` must come *before* `set_ranging_frequency_hz`,
because changing resolution clamps the max frequency (8×8 caps at 15 Hz, 4×4 at
60 Hz) and can silently override a frequency you set earlier.

---

## 6. Understanding the data

### 6a. What you get per frame

With `VL53L7CX_NB_TARGET_PER_ZONE = 1` and 8×8 resolution, `VL53L7CX_ResultsData`
is 1360 bytes:

| Field | Type | Count | Units | Notes |
|---|---|---|---|---|
| `silicon_temp_degc` | `int8_t` | 1 | °C | Sensor die temperature |
| `distance_mm` | `int16_t` | 64 | mm | Clamped to ≥ 0 |
| `target_status` | `uint8_t` | 64 | — | **5 and 9 mean valid**; 255 = no target |
| `range_sigma_mm` | `uint16_t` | 64 | mm | Std-dev estimate — your confidence metric |
| `signal_per_spad` | `uint32_t` | 64 | kcps/SPAD | Return strength |
| `ambient_per_spad` | `uint32_t` | 64 | kcps/SPAD | Background light |
| `nb_target_detected` | `uint8_t` | 64 | — | 0 → status forced to 255 |
| `nb_spads_enabled` | `uint32_t` | 64 | — | Diagnostic |
| `reflectance` | `uint8_t` | 64 | % | Estimated target reflectance |
| `motion_indicator` | struct | 1 | — | 32-entry motion array |

### 6b. Zone indexing

Zones are a flat array of 64. **Do not assume an orientation.** Determine it
empirically: hold a card over one corner of the sensor and see which indices
change. Then write the mapping down and never guess again.

For multi-target configs, zone `z` target `t` is at index
`z * VL53L7CX_NB_TARGET_PER_ZONE + t`.

### 6c. Filtering — this is where precision actually comes from

Never consume `distance_mm` unconditionally. Gate on three things:

```c
bool zone_valid(const VL53L7CX_ResultsData *r, int i)
{
    if (r->nb_target_detected[i] == 0)        return false;
    if (r->target_status[i] != 5 &&
        r->target_status[i] != 9)             return false;
    if (r->range_sigma_mm[i]  > 15)           return false;  /* tune this */
    if (r->signal_per_spad[i] < 100)          return false;  /* tune this */
    return true;
}
```

Status 5 is the highest-confidence result; 9 is valid but with reduced
confidence. `range_sigma_mm` is the most useful single quality metric — a zone
reporting 400 mm ± 3 mm is trustworthy; the same zone at 400 mm ± 60 mm is not.

---

## 7. Getting to high precision

Roughly in order of impact.

### 7.1 Cross-talk calibration — do this if you use the cover glass

Your kit ships with a cover glass and 0.25/0.5/1 mm spacers. **Any cover glass
reflects some emitted light straight back into the receiver.** Uncalibrated,
this adds a phantom near-field return that corrupts every zone, worst at long
range and low reflectance.

```c
vl53l7cx_calibrate_xtalk(&dev, reflectance_pct, nb_samples, distance_mm);
```

Point at a flat target of known reflectance at a known distance (ST suggests a
grey target around 600 mm). Then `vl53l7cx_get_caldata_xtalk()` and store the
776-byte blob in flash so you can `vl53l7cx_set_caldata_xtalk()` at every boot
instead of recalibrating.

**Skipping this is the most common cause of "my readings are wrong and I don't
know why" with this sensor.**

### 7.2 Trade frame rate for precision

Photon statistics: precision scales with integration time. Every one of these
costs frequency and buys accuracy.

| Lever | Effect |
|---|---|
| Lower `ranging_frequency_hz` | More integration per frame. 5 Hz is much better than 15 Hz. |
| `VL53L7CX_RANGING_MODE_AUTONOMOUS` + `set_integration_time_ms()` | Explicit control, up to 1000 ms. Best precision available. |
| `VL53L7CX_RESOLUTION_4X4` | 4× the SPADs per zone → longer range, lower noise, 4× fewer zones. |

If you need maximum precision and can accept few zones: 4×4, autonomous mode,
integration time 100–1000 ms.

`VL53L7CX_RANGING_MODE_CONTINUOUS` keeps the VCSEL running and gives better
ambient-light immunity at higher power draw. Autonomous mode is lower power and
gives you the integration-time knob. Try both.

### 7.3 Quarter-millimetre output

The driver divides the firmware's native values down to integer units:

```c
p_results->distance_mm[i]     /= 4;      /* native is 1/4 mm  */
p_results->range_sigma_mm[i]  /= 128;    /* native is 1/128 mm */
p_results->signal_per_spad[i] /= 2048;
p_results->reflectance[i]     /= 2;
```

Defining **`VL53L7CX_USE_RAW_FORMAT`** in `platform.h` skips that conversion and
hands you quarter-millimetre distances and 1/128 mm sigma. If you are averaging
or filtering, this recovers real resolution that integer division throws away.

**Caveat:** every consumer of the data must then apply the scale itself. Decide
early and be consistent, because mixing the two conventions produces distances
off by 4× that look almost plausible.

### 7.4 Temporal filtering

The sensor's per-frame noise is largely uncorrelated. For a static scene,
averaging N frames reduces noise by roughly √N.

- **Simple moving average** per zone, 4–8 frames — easy, adds latency.
- **Exponential moving average**: `d = α·new + (1−α)·d`, α ≈ 0.3 — cheap, one
  value of state per zone.
- **Weighted by `1/sigma²`** — statistically optimal, and it automatically
  down-weights bad frames instead of letting them poison the average.

Never average across frames where `target_status` changed from valid to invalid;
reset the filter on that transition.

### 7.5 Other knobs

- `vl53l7cx_set_sharpener_percent()` — default 5 %. Raising it separates
  adjacent targets more aggressively at the cost of some signal.
- `vl53l7cx_set_target_order()` — `CLOSEST` vs `STRONGEST`. For obstacle
  detection you almost always want `CLOSEST`.
- `vl53l7cx_calibrate_xtalk` aside, offset calibration data is factory-programmed
  and read during `init`; you rarely need to touch it.
- Keep the sensor thermally stable. `silicon_temp_degc` is reported for a
  reason — large temperature swings shift readings.

---

## 8. Getting the data off the board

### 8.1 Bandwidth budget — do this arithmetic before choosing a format

At 8×8, 1 target/zone, sending distance (2 B) + status (1 B) + sigma (2 B):

```
64 zones × 5 B          = 320 B/frame
+ header/CRC            ≈ 336 B/frame
× 15 Hz                 = 5.0 kB/s = 50 kbit/s at 8N1
```

Add `signal_per_spad` (4 B) and `ambient_per_spad` (4 B) and you are at
~13 kB/s = 130 kbit/s — **already above 115200 baud.**

At 921600 baud a 336-byte frame takes ~3.6 ms, comfortably inside a 66 ms frame
period. At 115200 it takes ~29 ms — 44 % of your budget, spent blocking inside
`HAL_UART_Transmit`.

**Conclusion:** run the link at 921600, and use DMA for TX
(`HAL_UART_Transmit_DMA`) with a double buffer so ranging and transmission
overlap.

### 8.2 Format by purpose

**Human-readable (bring-up only).** An 8×8 grid, redrawn in place:

```
      0     1     2     3     4     5     6     7
 0   412   408   415     -   1204  1198   890   885
 1   410   405   409   411  1201  1195   888   883
 ...
```

Print `-` for invalid zones rather than a misleading number. Do not stream this
at 15 Hz — throttle to 2 Hz for eyeballing.

**CSV (logging and offline analysis).** One row per frame:

```
seq,timestamp_ms,temp_c,d0,s0,g0,d1,s1,g1,...,d63,s63,g63
```

Trivially importable into Python/pandas or Excel. Best format for tuning your
filter thresholds against recorded ground truth.

**Binary (production).** Framed, length-prefixed, checksummed:

```
 Offset  Size  Field
 0       2     Sync         0xA5 0x5A
 2       1     Version      0x01
 3       1     Frame type   0x01 = 8x8 ranging
 4       2     Payload len  (little endian)
 6       4     Sequence     (detects drops)
 10      4     Timestamp    HAL_GetTick() ms
 14      1     Temp °C      int8
 15      N     Payload      per-zone records
 15+N    2     CRC-16/CCITT over bytes 2..15+N-1
```

Per-zone record, 5 bytes packed:
```
 int16  distance_mm
 uint16 range_sigma_mm
 uint8  target_status
```

The sequence number is not optional — without it you cannot distinguish
"the sensor stopped" from "the link dropped a frame", and you will chase the
wrong bug.

**Consider COBS** (Consistent Overhead Byte Stuffing) to make `0x00` a
guaranteed frame delimiter. It costs ~1 byte per 254 and makes resynchronisation
after a dropped byte trivial, which matters on a link you will unplug constantly.

### 8.3 Beyond UART

- **USB CDC** — the F446RE has a USB OTG FS peripheral. Full-speed USB gives
  you ~1 MB/s and removes the baud-rate ceiling entirely. More setup work.
- **SPI to a host** — if this eventually feeds another MCU rather than a PC.
- **SD card** — for untethered logging; write CSV or the binary frames directly.

### 8.4 Host-side

A ~30-line Python script with `pyserial` that parses the binary frames and
renders the 8×8 grid with `matplotlib`'s `imshow` will teach you more about
sensor behaviour in ten minutes than a week of reading numbers in a terminal.
Build it early.

---

## 9. Suggested order of work

1. Fix the debug channel (§1). Nothing else proceeds without it.
2. Update ST-LINK firmware; recover OpenOCD (§1c).
3. Settle on HSI, verify the clock by measurement (§2).
4. Confirm the 53L7A1 pin mapping (§3).
5. I²C1 init + bus scan → ACK at 0x29 (§4).
6. `vl53l7cx_is_alive` (§5a).
7. `vl53l7cx_init` (§5b) — expect this to be the hard one.
8. 4×4 at 5 Hz, print raw values, confirm they track a hand moving toward the
   sensor (§5c).
9. Add validity filtering (§6c).
10. Move to 8×8, add the binary protocol and a Python viewer (§8).
11. Cross-talk calibration with the cover glass (§7.1).
12. Tune precision: integration time, filtering, `USE_RAW_FORMAT` (§7).

---

## Appendix A — What `SysTick_Handler` does and why you need it

### The hardware

Every ARM Cortex-M has a **SysTick** peripheral built into the core itself —
not into STM32's peripheral set, but into the CPU. It is a 24-bit down-counter
with one job: count down from a reload value to zero, fire an interrupt, reload,
repeat.

It exists so that every Cortex-M has one timer the vendor cannot omit, which is
why RTOSes can be ported across chips without rewriting their tick source.

### What HAL does with it

`HAL_Init()` calls `HAL_InitTick(TICK_INT_PRIORITY)`, which does roughly:

```c
HAL_SYSTICK_Config(SystemCoreClock / 1000U);   /* fire every 1 ms */
```

So SysTick interrupts **1000 times per second**. Note the dependency on
`SystemCoreClock` — this is exactly why a wrong clock configuration breaks
timing everywhere, not just in the UART.

### What the handler does

```c
void SysTick_Handler(void)
{
    HAL_IncTick();
}
```

`HAL_IncTick()` increments a single global:

```c
static __IO uint32_t uwTick;
__weak void HAL_IncTick(void) { uwTick += uwTickFreq; }
```

That is the whole mechanism. `uwTick` is the HAL's only notion of elapsed time.

### Why everything depends on it

```c
__weak void HAL_Delay(uint32_t Delay)
{
    uint32_t tickstart = HAL_GetTick();     /* reads uwTick */
    uint32_t wait = Delay;
    if (wait < HAL_MAX_DELAY) wait += (uint32_t)(uwTickFreq);
    while ((HAL_GetTick() - tickstart) < wait) { }   /* spins */
}
```

`HAL_Delay` is a **busy-wait on `uwTick`**. If `uwTick` never changes, the
`while` condition is never satisfied and the function never returns. Not slow —
permanently. The MCU sits in that loop until reset.

The same applies to every HAL timeout: `HAL_I2C_Mem_Write(..., timeout)`,
`HAL_UART_Transmit(..., timeout)`, and the ULD's `VL53L7CX_WaitMs()` all measure
time by polling `HAL_GetTick()`.

### Why it is not there by default

The vendor startup file (`startup_stm32f446xx.s`) declares every interrupt
handler as a **weak symbol** aliased to `Default_Handler`:

```asm
    .weak   SysTick_Handler
    .thumb_set SysTick_Handler, Default_Handler
```

And `Default_Handler` is:

```asm
Infinite_Loop:
    b   Infinite_Loop
```

So the vector table is always populated — the CPU never jumps into unmapped
memory — but the default behaviour for any handler you forget is to hang.

STM32CubeIDE generates `stm32f4xx_it.c` containing `SysTick_Handler` for you.
**PlatformIO's `stm32cube` framework does not.** You are responsible for
providing it, and if you do not, the symptom is: the board flashes fine, `main`
starts, and it locks up on the very first `HAL_Delay()` with no error, no fault,
and no output.

Defining it in `main.c` provides a *strong* symbol that overrides the weak alias
at link time. That is all that is required — no registration, no vector table
edits.

### One gotcha worth knowing now

`__disable_irq()` — which the old `Error_Handler` and the current `Fault()` both
call — stops SysTick along with everything else. Inside those functions
`HAL_Delay()` would hang, which is precisely why they use raw `for` loops with
`volatile` counters instead. Keep that in mind before adding any HAL call to a
fault path.

---

## Appendix B — Reference documents

| Doc | Covers |
|---|---|
| **UM3067** | X-NUCLEO-53L7A1 getting started + **schematic** — your pin mapping |
| **UM3108** | X-CUBE-TOF1 software package for STM32CubeMX |
| **UM1724** | Nucleo-64 boards — solder bridges, VCP routing, LD1/LD2/LD3 |
| **DS14012** | VL53L7CX datasheet — timing, optical, electrical specs |
| **UM2884** | VL53L7CX ULD driver API reference |
| **STSW-LINK007** | ST-LINK firmware upgrade utility |
| **STSW-LINK009** | ST-LINK USB driver (Windows) |
