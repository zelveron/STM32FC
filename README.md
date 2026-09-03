# STM32FC — STM32F407VET6 Flight-Controller Sensor Board

Firmware + host tools for a custom **STM32F407VET6** board that reads four
sensors, streams tagged CSV over **USB CDC**, logs to an **SD card**, and shows
everything in a **tkinter GUI** on a Raspberry Pi 5.

> **This README is written for future AI coding agents.** It records every
> gotcha that was learned the hard way (marginal solder joints, DFU quirks,
> sensor protocol traps). Read the "Known issues" section before touching code.

## Sensors & status (2026-09-03)

| Sensor  | Bus / pins                        | Status                              |
|---------|-----------------------------------|-------------------------------------|
| BMI323  | SPI1 PA4=CS PA5=SCK PA6=MISO PA7=MOSI | ✅ working — **bit-bang SPI only** |
| BMP581  | I2C1 PB6=SCL PB7=SDA, addr 0x47  | ✅ working (pressure/temp/alt)     |
| uBlox GNSS | USART1 PA9=TX PA10=RX (NMEA) | ✅ working — NMEA at 9600 baud; **no fix yet indoors (0 sats)** |
| ALS31300 (Hall) | I2C1 PB6/PB7, addr **0x7E** | ⚠️ ACKs address but NACKs all register writes — **hardware issue, not yet readable** |

## Hardware pin map (verified)

| Device     | Signal     | STM32 pin | Notes                                   |
|------------|------------|-----------|-----------------------------------------|
| BMI323     | SCK        | PA5       | SPI mode 0, bit-bang (~200 kHz)         |
| BMI323     | SDO/MISO   | PA6       |                                         |
| BMI323     | SDI/MOSI   | PA7       |                                         |
| BMI323     | CS         | PA4       |                                         |
| BMP581     | SCL        | PB6       | I2C1                                    |
| BMP581     | SDA        | PB7       | I2C1, 7-bit addr **0x47**               |
| uBlox GNSS | TX (MCU)   | PA9       | USART1_TX (PA9 is **TX-only** on F407)  |
| uBlox GNSS | RX (MCU)   | PA10      | USART1_RX (PA10 is **RX-only**)         |
| uBlox GNSS | (no enable)| —         | module is always-on; **no enable pin**  |
| ALS31300   | SCL/SDA    | PB6/PB7   | same I2C bus as BMP581, addr 0x7E       |
| SD card    | D0..D3, CK, CMD | PC8..PC12, PD2 | SDIO 4-bit (see SD section)      |
| USB CDC    | D+/D-      | PA12/PA11 | native USB, `SerialUSB`                 |

## ⚠️ CRITICAL — BMI323 must use bit-bang SPI

The BMI323's solder joints on this board are **marginal**. Hardware SPI fails
even at 1 MHz (chip ID reads `0xFF` = MISO floating). It only works with the
**bit-bang SPI** in `src/main.cpp`:

- `bmi_bb_xfer()` — bit-bangs one byte (mode 0, `delayMicroseconds(2)` per edge)
- `bmi3_spi_read()` / `bmi3_spi_write()` — CS + bit-bang, wired to the Bosch driver
- `bmi_raw_chip_id()` — raw reg 0x00 read for diagnostics (`0x43` = present)

Do **not** replace these with hardware SPI until the solder joints are reflowed.

## ⚠️ CRITICAL — uBlox UART wiring must be CROSSED

On STM32F407 USART1 has **no remap**:
- `PA9` = USART1_TX (can only transmit)
- `PA10` = USART1_RX (can only receive)

The uBlox must be wired **crossed**: `uBlox TX → PA10`, `uBlox RX → PA9`.
If wired straight (TX→TX, RX→RX) the MCU receives nothing (we proved this by
edge-counting both pins: signal was stuck on PA9, a TX-only pin). There is **no
enable pin** — the module is powered on whenever the board is powered.

## Build

```bash
cd ~/Desktop/STM32FC
pio run
```

Board = `black_f407ve`, framework = Arduino (`Arduino_Core_STM32`), USB CDC
enabled via `-D PIO_FRAMEWORK_ARDUINO_ENABLE_CDC`. Output:
`.pio/build/black_f407ve/firmware.bin`.

## Flash (DFU, STM32 ROM bootloader)

The board flashes over USB DFU (`0483:df11`). **Enumeration is flaky on the
Raspberry Pi** — `dfu-util -l` often shows nothing (kernel logs
`device descriptor read/64, error -110`).

1. Set **BOOT0 = 1** and press **reset** (keep BOOT0 high).
2. If `dfu-util -l` is empty, **unplug and replug the USB cable** while BOOT0
   stays high. Check the device actually shows `0483:df11`, not `0483:5740`
   (5740 = the running app, meaning BOOT0 is not actually high).
3. Flash:
   ```bash
   dfu-util -a 0 -s 0x08000000:leave -D .pio/build/black_f407ve/firmware.bin
   ```
4. Set **BOOT0 = 0** and press **reset** to run.

## Serial protocol (USB CDC, 115200 baud, tagged CSV)

Firmware streams these lines; the GUI parses them by leading tag:

```
BMP,<pressure_hPa>,<temp_C>,<altitude_m>
BMI,<acc_x>,<acc_y>,<acc_z>,<gyr_x>,<gyr_y>,<gyr_z>     (g, deg/s)
ATT,<roll_deg>,<pitch_deg>,<yaw_deg>
GPS_STAT,<fix>,<sats>,<time_HH:MM:SS>,<speed_kmh>       (1 Hz, even without fix)
GPS,<lat>,<lon>,<alt_m>,<sats>,<fix>,<time>,<speed_kmh> (only with a position fix)
GPS_RAW,<last NMEA sentence>                             (1 Hz debug)
GPS_DBG,<rx_bytes>,<baud>,<locked>                       (2 s debug)
GPS_FIRST,<len>,<hex boot bytes>                         (3 s debug)
PA_EDGES,<pa9_edges>,<pa9_lvl>,<pa10_edges>,<pa10_lvl>   (4 s wiring debug)
ALS_STATUS,0|1   ALS_RAW,<hex>   ALS,<x>,<y>,<z>,<temp_c>
BMI_STATUS,0|1   BMI_RAW,0xNN
I2C_SCAN,<hex addresses...>
SD_STATUS,1|<file>   SD_DBG,<ok>,<file>
```

- BMI323: accel ±4 g, gyro ±2000 dps, ODR 200 Hz (sampled ~100 Hz).
- BMP581: 16x pressure oversampling, forced mode, ~10 Hz.
- GPS speed is computed from `$GxRMC` knots × 1.852 → km/h.
- GPS time is **UTC** (`HH:MM:SS` from `$GxGGA` / `$GxRMC`).

## GUI

```bash
python3 tools/gui.py              # auto-detect /dev/ttyACM0
python3 tools/gui.py --port /dev/ttyACM0
```

Sections: BMI323 status, BMP581 (p/t/alt), BMI323 (accel/gyro), uBlox GNSS
(position/alt/sats/fix/time/speed), Attitude (roll/pitch/yaw + artificial
horizon canvas). It auto-reconnects and has a data watchdog.

## SD card logging

SDIO 4-bit, FatFs. Firmware writes `FLTxxxxx.CSV` (next free index each boot):

```
t_ms,ax,ay,az,gx,gy,gz,roll,pitch,yaw,press_hPa,temp_c,alt_m,gps_time,gps_sats,gps_speed_kmh
```

Logged at ~50 Hz, flushed every 1 s. Vendored libs `lib/STM32SD/` + `lib/FatFs/`
have critical fixes (see Known issues) — do not regenerate them from upstream.

## Known issues & lessons (READ BEFORE CODING)

1. **BMI323 marginal solder** → bit-bang SPI only (see above). Reflow
   VDD/VDDIO/GND/SDO if hardware SPI is ever wanted.
2. **SD hang** → the stock STM32 HAL SD timeout was `100000000 ms` (≈27.8 h),
   which looked like an infinite hang. Fixed in vendored code:
   `lib/STM32SD/src/bsp_sd.h` `SD_DATATIMEOUT` → 2000;
   `lib/STM32SD/src/bsp_sd.c` `SD_CLK_DIV` → 10U (4 MHz) and `SD_BUS_WIDE` → 1B.
   Also `lib/FatFs/` **must** include the `ffsystem/` folder or the build fails.
3. **ALS31300** at 0x7E ACKs its address but **NACKs every register/data byte**
   (Wire error 3). CAC unlock (0x35 ← 0x2C413534), op-mode 0x27 wake, and a
   10 kHz slow clock all fail. The mainline Linux driver wakes it with a plain
   0x27 write (no CAC), so a healthy chip should respond — this one is a
   hardware/power/wiring problem, not firmware. Registers: 0x27 op mode
   (bits[1:0] 0=active), 0x28/0x29 = 32-bit X/Y/Z/temp data.
4. **uBlox must be crossed-wired** (TX→PA10, RX→PA9). Without an antenna it
   still sends `$GPTXT` boot messages + 1 Hz NMEA with empty fix (time fills in
   as soon as any satellite is heard). Indoors you may see `sats=0` forever.
5. **DFU enumeration is flaky** — see Flash section. The `:leave` flag makes the
   board run the app right after flashing.
6. **Serial capture contention** — `tools/gui.py` and any `cat /dev/ttyACM0`
   both open the same CDC port and split the stream. Kill the GUI before doing a
   clean `cat` capture. `tools/capture.py` survives USB re-enumeration.
7. **Boot prints are lost** unless the firmware waits for the USB host — the
   setup() has a bounded `while (!SerialUSB)` wait for this reason.
8. **GPS baud auto-detect** locks the first baud at which a checksum-valid NMEA
   sentence arrives (it does **not** wait for a satellite fix), then stops
   cycling through the 9 candidate bauds.

## Project layout

```
platformio.ini          board black_f407ve, Arduino, USB CDC, dfu upload
src/main.cpp            all firmware (sensors, attitude, GPS, SD logging)
lib/bmi323/             vendored Bosch BMI323 API
lib/bmp5/               vendored Bosch BMP5 API (BMP581)
lib/STM32SD/            vendored stm32duino SD (with timeout/clk fixes)
lib/FatFs/              vendored FatFs (must keep ffsystem/)
tools/gui.py            tkinter live GUI
tools/capture.py        robust serial capture (survives re-enumeration)
```

## Git

Remote: `https://github.com/zelveron/STM32FC` (branch `main`). Author identity
`zelveron <zelveron@users.noreply.github.com>` is set repo-locally. Push with
`git push origin main` — credentials are already cached on the Pi.
