# STM32F407 + BMI323 + BMP581 — Working Reference

Firmware and host tools for the custom **STM32F407VET6** board:

- **BMI323** 6-axis IMU (accel + gyro) over **SPI1** (bit-bang)
- **BMP581** pressure / temperature / altitude over **I2C1**
- streamed to a Raspberry Pi 5 over **USB CDC** (`SerialUSB`)

This is the **known-good** configuration (verified 2026-08-27).

## Hardware wiring (verified)

| Sensor | Signal   | STM32F407 pin | Note           |
|--------|----------|---------------|----------------|
| BMI323 | SCK      | PA5           | SPI1, mode 0   |
| BMI323 | SDO/MISO | PA6           |                |
| BMI323 | SDI/MOSI | PA7           |                |
| BMI323 | CS       | PA4           |                |
| BMP581 | SCL      | PB6           | I2C1           |
| BMP581 | SDA      | PB7           | I2C1 @ 0x47    |
| USB    | D+/D-    | PA12 / PA11   | native USB CDC |

BMI323 chip ID register `0x00` reads **0x43**.

## CRITICAL: read the BMI323 with bit-bang SPI

The BMI323 connection on this board is **marginal** (weak solder joint).
Hardware SPI fails at 5.25 MHz and even 1 MHz (chip ID reads `0xFF` = MISO
floating). It only works reliably with **bit-bang SPI at ~200 kHz**.

The working bit-bang implementation is in `src/main.cpp`:

- `bmi_bb_xfer()` — bit-bangs one byte (mode 0, sample MISO on rising SCK)
- `bmi3_spi_read()` / `bmi3_spi_write()` — CS + bit-bang, fed to the Bosch driver
- `bmi_raw_chip_id()` — raw `0x00` register read for diagnostics

Do **not** swap these for fast hardware SPI until the solder joint is reflowed.

Diagnostic tags printed by the firmware:

- `BMI_STATUS,1` — BMI323 initialized OK
- `BMI_RAW,0x43` — raw chip-id read = BMI323 present
- `BMI_RAW,0xFF` — MISO floating = no device / bad connection

## Serial output format (USB CDC, 115200 baud)

```
BMP,<pressure_hPa>,<temp_C>,<altitude_m>
BMI,<acc_x>,<acc_y>,<acc_z>,<gyr_x>,<gyr_y>,<gyr_z>   (g, deg/s)
BMI_STATUS,0|1
BMI_RAW,0xNN
```

- BMI accel range ±4 g, gyro range ±2000 dps, ODR 200 Hz (firmware samples ~100 Hz).
- BMP581: pressure 16x oversampling, forced mode, ~10 Hz.

## Project layout

```
platformio.ini          board: black_f407ve, framework: Arduino, USB CDC enabled
src/main.cpp            firmware: BMP581 (I2C) + BMI323 (bit-bang SPI) streamer
lib/bmi323/             vendored Bosch BMI323 API (bmi3.c + bmi323.c)
lib/bmp5/               vendored Bosch BMP5 API (BMP581)
tools/gui.py            tkinter live GUI (auto-reconnect + data watchdog)
tools/imureader.py      console reader / logger
```

## Build

```bash
cd ~/Desktop/STM32FC
pio run
```

## Flash (DFU)

The board uses the STM32 ROM bootloader (USB DFU, `0483:df11`):

1. Set **BOOT0 = 1**, press reset, **keep BOOT0 high**.
2. DFU enumeration is flaky on the Pi — if `dfu-util -l` shows nothing,
   **replug the USB cable** (with BOOT0 still high) and wait a few seconds.
3. Flash:
   ```bash
   dfu-util -a 0 -s 0x08000000:leave -D .pio/build/black_f407ve/firmware.bin
   ```
4. Set **BOOT0 = 0** and reset to run.

## Read data

```bash
python3 tools/gui.py              # GUI (auto-detect /dev/ttyACM0)
python3 tools/imureader.py        # console
```

## Other pins on the board

- `UBLOX_EN` = PE10, `BMP_EN` = PB11, `BMP_EN_ALT` = PE11 (driven HIGH in setup)
- PC14 (OSC32 pin) used as ALS enable — driven via `GPIOC` registers.
