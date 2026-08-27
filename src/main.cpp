/**
 * STM32F407VET6 - BMP581 + BMI323 real-time streamer
 *
 *   BMP581 : I2C1  PB6=SCL, PB7=SDA @0x47  (pressure / temp / altitude)
 *   BMI323 : SPI1  PA5=SCK, PA6=MISO, PA7=MOSI, PA4=CS (accel / gyro)
 *
 * Output over USB CDC ("SerialUSB"), tagged CSV lines:
 *   BMP,<pressure_hPa>,<temperature_C>,<altitude_m>
 *   BMI,<acc_x>,<acc_y>,<acc_z>,<gyr_x>,<gyr_y>,<gyr_z>   (g, deg/s)
 *   BMI_STATUS,0|1       (0 = not detected, 1 = detected)
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <string.h>

#include "bmp5.h"
#include "bmp5_defs.h"
#include "bmi323.h"

#define UBLOX_EN   PE10
#define BMP_EN     PB11
#define BMP_EN_ALT PE11
#define BMI_CS     PA4
#define BMI_SCK    PA5
#define BMI_MISO   PA6
#define BMI_MOSI   PA7

static const uint8_t BMP5_ADDR = 0x47;

/* ------------------------------------------------------------------------- */
/* BMP581 (I2C)                                                              */
/* ------------------------------------------------------------------------- */

static int8_t bmp5_i2c_read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;
    Wire.beginTransmission(BMP5_ADDR);
    Wire.write(reg_addr);
    if (Wire.endTransmission(false) != 0) return BMP5_E_COM_FAIL;
    if (Wire.requestFrom((uint8_t)BMP5_ADDR, (uint8_t)len) != len) return BMP5_E_COM_FAIL;
    for (uint32_t i = 0; i < len; i++) read_data[i] = Wire.read();
    return BMP5_INTF_RET_SUCCESS;
}

static int8_t bmp5_i2c_write(uint8_t reg_addr, const uint8_t *write_data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;
    Wire.beginTransmission(BMP5_ADDR);
    Wire.write(reg_addr);
    for (uint32_t i = 0; i < len; i++) Wire.write(write_data[i]);
    if (Wire.endTransmission() != 0) return BMP5_E_COM_FAIL;
    return BMP5_INTF_RET_SUCCESS;
}

static void bmp5_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    delayMicroseconds(period);
}

static struct bmp5_dev bmp_dev;
static struct bmp5_osr_odr_press_config bmp_cfg;

static bool bmp5_begin(void)
{
    memset(&bmp_dev, 0, sizeof(bmp_dev));
    bmp_dev.intf     = BMP5_I2C_INTF;
    bmp_dev.read     = bmp5_i2c_read;
    bmp_dev.write    = bmp5_i2c_write;
    bmp_dev.delay_us = bmp5_delay_us;

    if (bmp5_init(&bmp_dev) != BMP5_OK) return false;

    bmp_cfg.osr_t    = BMP5_OVERSAMPLING_1X;
    bmp_cfg.osr_p    = BMP5_OVERSAMPLING_16X;
    bmp_cfg.press_en = BMP5_ENABLE;
    bmp_cfg.odr      = BMP5_ODR_100_2_HZ;

    if (bmp5_set_osr_odr_press_config(&bmp_cfg, &bmp_dev) != BMP5_OK) return false;
    return true;
}

/* ------------------------------------------------------------------------- */
/* BMI323 (SPI)                                                              */
/* ------------------------------------------------------------------------- */

/* Bit-bang one byte over SPI, mode 0 (sample MISO on rising SCK). */
static uint8_t bmi_bb_xfer(uint8_t out)
{
    uint8_t in = 0;
    for (int i = 7; i >= 0; i--)
    {
        digitalWrite(BMI_MOSI, (out >> i) & 1);
        digitalWrite(BMI_SCK, HIGH);
        delayMicroseconds(2);
        if (digitalRead(BMI_MISO)) in |= (1 << i);
        digitalWrite(BMI_SCK, LOW);
        delayMicroseconds(2);
    }
    return in;
}

static int8_t bmi3_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;
    if (length == 0) return BMI3_INTF_RET_SUCCESS;

    digitalWrite(BMI_CS, LOW);
    delayMicroseconds(1);
    bmi_bb_xfer(reg_addr);          /* driver already set the 0x80 read bit */
    for (uint32_t i = 0; i < length; i++) reg_data[i] = bmi_bb_xfer(0x00);
    delayMicroseconds(1);
    digitalWrite(BMI_CS, HIGH);
    return BMI3_INTF_RET_SUCCESS;
}

static int8_t bmi3_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    (void)intf_ptr;
    if (length == 0) return BMI3_INTF_RET_SUCCESS;

    digitalWrite(BMI_CS, LOW);
    delayMicroseconds(1);
    bmi_bb_xfer(reg_addr);          /* driver already applied the 0x7F write mask */
    for (uint32_t i = 0; i < length; i++) bmi_bb_xfer(reg_data[i]);
    delayMicroseconds(1);
    digitalWrite(BMI_CS, HIGH);
    return BMI3_INTF_RET_SUCCESS;
}

static void bmi3_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    delayMicroseconds(period);
}

static struct bmi3_dev bmi_dev;
static bool bmi_ready = false;

/* Raw single-transaction CHIP_ID read (no config upload) for diagnostics. */
static uint8_t bmi_raw_chip_id(void)
{
    uint8_t b[2] = { 0, 0 };
    digitalWrite(BMI_CS, LOW);
    delayMicroseconds(1);
    bmi_bb_xfer(0x80);          /* read register 0x00 */
    b[0] = bmi_bb_xfer(0x00);   /* dummy byte */
    b[1] = bmi_bb_xfer(0x00);   /* chip_id (0x43 for BMI323) */
    delayMicroseconds(1);
    digitalWrite(BMI_CS, HIGH);
    return b[1];
}

static bool bmi323_begin(void)
{
    memset(&bmi_dev, 0, sizeof(bmi_dev));
    bmi_dev.intf           = BMI3_SPI_INTF;
    bmi_dev.read           = bmi3_spi_read;
    bmi_dev.write          = bmi3_spi_write;
    bmi_dev.delay_us       = bmi3_delay_us;
    bmi_dev.read_write_len = 8;

    if (bmi323_init(&bmi_dev) != BMI3_OK) return false;

    struct bmi3_sens_config cfg[2];
    cfg[0].type = BMI323_ACCEL;
    cfg[1].type = BMI323_GYRO;
    if (bmi323_get_sensor_config(cfg, 2, &bmi_dev) != BMI3_OK) return false;

    cfg[0].cfg.acc.odr      = BMI3_ACC_ODR_200HZ;
    cfg[0].cfg.acc.range    = BMI3_ACC_RANGE_4G;
    cfg[0].cfg.acc.bwp      = BMI3_ACC_BW_ODR_QUARTER;
    cfg[0].cfg.acc.avg_num  = BMI3_ACC_AVG4;
    cfg[0].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

    cfg[1].cfg.gyr.odr      = BMI3_GYR_ODR_200HZ;
    cfg[1].cfg.gyr.range    = BMI3_GYR_RANGE_2000DPS;
    cfg[1].cfg.gyr.bwp      = BMI3_GYR_BW_ODR_HALF;
    cfg[1].cfg.gyr.avg_num  = BMI3_GYR_AVG1;
    cfg[1].cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;

    if (bmi323_set_sensor_config(cfg, 2, &bmi_dev) != BMI3_OK) return false;

    return true;
}

/* ------------------------------------------------------------------------- */
/* setup / loop                                                              */
/* ------------------------------------------------------------------------- */

void setup(void)
{
    pinMode(UBLOX_EN, OUTPUT);   digitalWrite(UBLOX_EN, HIGH);
    pinMode(BMP_EN_ALT, OUTPUT); digitalWrite(BMP_EN_ALT, HIGH);
    pinMode(BMP_EN, OUTPUT);     digitalWrite(BMP_EN, HIGH);

    /* PC14: ALS enable (OSC32 pin - drive via registers) */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER = (GPIOC->MODER & ~(3UL << 28)) | (1UL << 28);
    GPIOC->BSRR = (1UL << 14);

    pinMode(BMI_CS, OUTPUT);   digitalWrite(BMI_CS, HIGH);
    pinMode(BMI_SCK, OUTPUT);  digitalWrite(BMI_SCK, LOW);
    pinMode(BMI_MOSI, OUTPUT); digitalWrite(BMI_MOSI, LOW);
    pinMode(BMI_MISO, INPUT_PULLUP);

    Wire.setSCL(PB6);
    Wire.setSDA(PB7);
    Wire.begin();

    SerialUSB.begin();
    delay(300);
    SerialUSB.println(F("boot: BMP581 + BMI323 streamer"));
}

void loop(void)
{
    /* --- BMI323 presence detection (retry every 1 s until found) --------- */
    static uint32_t last_bmi_retry = 0;
    if (!bmi_ready && (millis() - last_bmi_retry) >= 1000)
    {
        last_bmi_retry = millis();
        if (bmi323_begin())
        {
            bmi_ready = true;
            SerialUSB.println(F("BMI_STATUS,1"));
        }
        else
        {
            SerialUSB.println(F("BMI_STATUS,0"));
            uint8_t raw = bmi_raw_chip_id();
            SerialUSB.print(F("BMI_RAW,0x"));
            SerialUSB.println(raw, HEX);
        }
    }

    /* --- BMI323 streaming at ~100 Hz ------------------------------------- */
    static uint32_t last_bmi = 0;
    if (bmi_ready && (millis() - last_bmi) >= 10)
    {
        last_bmi = millis();

        struct bmi3_sensor_data data[2];
        data[0].type = BMI323_ACCEL;
        data[1].type = BMI323_GYRO;
        if (bmi323_get_sensor_data(data, 2, &bmi_dev) == BMI3_OK)
        {
            float ax = (float)data[0].sens_data.acc.x * 4.0f / 32768.0f;
            float ay = (float)data[0].sens_data.acc.y * 4.0f / 32768.0f;
            float az = (float)data[0].sens_data.acc.z * 4.0f / 32768.0f;
            float gx = (float)data[1].sens_data.gyr.x * 2000.0f / 32768.0f;
            float gy = (float)data[1].sens_data.gyr.y * 2000.0f / 32768.0f;
            float gz = (float)data[1].sens_data.gyr.z * 2000.0f / 32768.0f;

            SerialUSB.print(F("BMI,"));
            SerialUSB.print(ax, 4);
            SerialUSB.print(',');
            SerialUSB.print(ay, 4);
            SerialUSB.print(',');
            SerialUSB.print(az, 4);
            SerialUSB.print(',');
            SerialUSB.print(gx, 2);
            SerialUSB.print(',');
            SerialUSB.print(gy, 2);
            SerialUSB.print(',');
            SerialUSB.println(gz, 2);
        }
        else
        {
            /* Read failed - drop out of ready so we re-initialize. */
            bmi_ready = false;
            SerialUSB.println(F("BMI_STATUS,0"));
        }
    }

    /* --- BMP581 streaming at ~10 Hz -------------------------------------- */
    static uint32_t last_bmp = 0;
    if ((millis() - last_bmp) >= 100)
    {
        last_bmp = millis();

        static bool bmp_ok = false;
        if (!bmp_ok)
        {
            bmp_ok = bmp5_begin();
        }

        if (bmp_ok)
        {
            bmp5_set_power_mode(BMP5_POWERMODE_FORCED, &bmp_dev);
            delay(30);

            struct bmp5_sensor_data data;
            if (bmp5_get_sensor_data(&data, &bmp_cfg, &bmp_dev) == BMP5_OK)
            {
                float p = data.pressure;
                float t = data.temperature;
                float alt = 44330.0f * (1.0f - powf(p / 101325.0f, 0.1902632f));

                SerialUSB.print(F("BMP,"));
                SerialUSB.print(p / 100.0f, 3);
                SerialUSB.print(',');
                SerialUSB.print(t, 2);
                SerialUSB.print(',');
                SerialUSB.println(alt, 2);
            }
        }
    }
}
