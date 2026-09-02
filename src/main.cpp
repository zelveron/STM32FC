/**
 * STM32F407VET6 - BMP581 + BMI323 real-time streamer
 *
 *   BMP581 : I2C1  PB6=SCL, PB7=SDA @0x47  (pressure / temp / altitude)
 *   BMI323 : SPI1  PA5=SCK, PA6=MISO, PA7=MOSI, PA4=CS (accel / gyro)
 *   uBlox  : USART1 PA9=TX, PA10=RX (NMEA), en PE10
 *
 * Output over USB CDC ("SerialUSB"), tagged CSV lines:
 *   BMP,<pressure_hPa>,<temperature_C>,<altitude_m>
 *   BMI,<acc_x>,<acc_y>,<acc_z>,<gyr_x>,<gyr_y>,<gyr_z>   (g, deg/s)
 *   GPS,<lat>,<lon>,<alt_m>,<sats>,<fix>
 *   ATT,<roll_deg>,<pitch_deg>,<yaw_deg>
 *   SD_STATUS,1|<logfile>  (0 = SD init/open failed)
 *   BMI_STATUS,0|1       (0 = not detected, 1 = detected)
 *
 * Sensor data is also logged to the SD card (SDIO, FatFs) as FLTxxxxx.CSV.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "bmp5.h"
#include "bmp5_defs.h"
#include "bmi323.h"
#include <STM32SD.h>

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
/* ALS31300 3-axis Hall-effect sensor (I2C1 @0x60)                           */
/* ------------------------------------------------------------------------- */

#define ALS_ADDR 0x7E   /* found by I2C scan (126 decimal) */

static void als_write(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(ALS_ADDR);
    Wire.write(reg);
    for (uint8_t i = 0; i < len; i++) Wire.write(buf[i]);
    Wire.endTransmission();
}

static int  als_read_fail_stage = 0;   /* 0=ok, 1=write NACK, 2=short read */
static int  als_read_got = 0;          /* bytes received on short read */

static bool als_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(ALS_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) { als_read_fail_stage = 1; return false; }
    int n = Wire.requestFrom((uint8_t)ALS_ADDR, len);
    if (n != len) { als_read_fail_stage = 2; als_read_got = n; return false; }
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    als_read_fail_stage = 0;
    return true;
}

/* One-time diagnostic: verify wake write + reads after als_begin(). */
static void als_debug_probe(void)
{
    uint8_t b[8];
    uint8_t v[4] = { 0x00, 0x00, 0x00, 0x00 };
    uint8_t cac_msb[4] = { 0x2C, 0x41, 0x35, 0x34 };
    uint8_t cac_lsb[4] = { 0x34, 0x35, 0x41, 0x2C };
    int r;

    /* Test 1: CAC unlock, MSB first, to 0x35. */
    Wire.beginTransmission(ALS_ADDR); Wire.write(0x35);
    for (int i = 0; i < 4; i++) Wire.write(cac_msb[i]);
    r = Wire.endTransmission();
    SerialUSB.print(F("ALS_PRB_CAC_MSB,")); SerialUSB.println(r);

    /* Test 2: CAC unlock, LSB first, to 0x35. */
    Wire.beginTransmission(ALS_ADDR); Wire.write(0x35);
    for (int i = 0; i < 4; i++) Wire.write(cac_lsb[i]);
    r = Wire.endTransmission();
    SerialUSB.print(F("ALS_PRB_CAC_LSB,")); SerialUSB.println(r);

    /* Test 3: write op mode 0x27 (active). */
    Wire.beginTransmission(ALS_ADDR); Wire.write(0x27);
    for (int i = 0; i < 4; i++) Wire.write(v[i]);
    r = Wire.endTransmission();
    SerialUSB.print(F("ALS_PRB_WAKE,")); SerialUSB.println(r);
    delayMicroseconds(600);

    /* Test 4: read op mode 0x27 (4 bytes). */
    SerialUSB.print(F("ALS_PRB_R27_4,"));
    if (als_read(0x27, b, 4))
    {
        for (int i = 0; i < 4; i++) { if (b[i] < 0x10) SerialUSB.print('0'); SerialUSB.print(b[i], HEX); if (i < 3) SerialUSB.print(','); }
        SerialUSB.println();
    }
    else
    {
        SerialUSB.print(F("FAIL,")); SerialUSB.print(als_read_fail_stage); SerialUSB.print(','); SerialUSB.println(als_read_got);
    }

    /* Test 5: read data 0x28 (8 bytes). */
    SerialUSB.print(F("ALS_PRB_R28_8,"));
    if (als_read(0x28, b, 8))
    {
        for (int i = 0; i < 8; i++) { if (b[i] < 0x10) SerialUSB.print('0'); SerialUSB.print(b[i], HEX); if (i < 7) SerialUSB.print(','); }
        SerialUSB.println();
    }
    else
    {
        SerialUSB.print(F("FAIL,")); SerialUSB.print(als_read_fail_stage); SerialUSB.print(','); SerialUSB.println(als_read_got);
    }

    /* --- Retry at a slow I2C clock (marginal-solder diagnosis) ---------- */
    Wire.setClock(10000);

    Wire.beginTransmission(ALS_ADDR); Wire.write(0x27);
    for (int i = 0; i < 4; i++) Wire.write(v[i]);
    r = Wire.endTransmission();
    SerialUSB.print(F("ALS_PRB_10K_WAKE,")); SerialUSB.println(r);
    delayMicroseconds(600);

    SerialUSB.print(F("ALS_PRB_10K_R28_8,"));
    if (als_read(0x28, b, 8))
    {
        for (int i = 0; i < 8; i++) { if (b[i] < 0x10) SerialUSB.print('0'); SerialUSB.print(b[i], HEX); if (i < 7) SerialUSB.print(','); }
        SerialUSB.println();
    }
    else
    {
        SerialUSB.print(F("FAIL,")); SerialUSB.print(als_read_fail_stage); SerialUSB.print(','); SerialUSB.println(als_read_got);
    }

    Wire.setClock(100000);
}

static bool als_present = false;

static void als_begin(void)
{
    /* Quick ACK check: is the ALS31300 on the bus? */
    Wire.beginTransmission(ALS_ADDR);
    als_present = (Wire.endTransmission() == 0);
    if (!als_present) return;

    /* Unlock customer access (CAC 0x2C413534, MSB first) at register 0x35. */
    static const uint8_t cac[4] = { 0x2C, 0x41, 0x35, 0x34 };
    als_write(0x35, cac, 4);

    /* Wake from sleep: operating-mode register 0x27 is a 32-bit register.
       bits[1:0] = 0 -> active mode. Write all 4 bytes (MSB first). */
    uint8_t v[4] = { 0x00, 0x00, 0x00, 0x00 };
    als_write(0x27, v, 4);

    /* Exit-sleep time == power-on delay time (600 us per datasheet). */
    delayMicroseconds(600);
}

static bool als_get(int16_t *x, int16_t *y, int16_t *z, int16_t *t, uint8_t *raw)
{
    uint8_t b[8];
    if (!als_read(0x28, b, 8)) return false;
    if (raw) for (int i = 0; i < 8; i++) raw[i] = b[i];

    int16_t rx = (int16_t)(((uint16_t)b[0] << 4) | (b[4] & 0x0F));
    if (rx & 0x0800) rx |= (int16_t)0xF000;
    int16_t ry = (int16_t)(((uint16_t)b[1] << 4) | (b[5] & 0x0F));
    if (ry & 0x0800) ry |= (int16_t)0xF000;
    int16_t rz = (int16_t)(((uint16_t)b[2] << 4) | (b[6] & 0x0F));
    if (rz & 0x0800) rz |= (int16_t)0xF000;
    int16_t rt = (int16_t)(((uint16_t)(b[3] & 0x3F) << 6) | (b[7] & 0x3F));

    *x = rx; *y = ry; *z = rz; *t = rt;
    return true;
}

/* Scan the I2C bus and report ACKing 7-bit addresses (0x08..0x7F). */
static void i2c_scan(void)
{
    SerialUSB.print(F("I2C_SCAN,"));
    bool first = true;
    for (uint16_t addr = 8; addr <= 0x7F; addr++)
    {
        Wire.beginTransmission((uint8_t)addr);
        if (Wire.endTransmission() == 0)
        {
            if (!first) SerialUSB.print(',');
            SerialUSB.print(addr, HEX);
            first = false;
        }
    }
    SerialUSB.println();
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
/* Attitude estimate (complementary filter — ArduPilot DCM/EKF foundation)   */
/* ------------------------------------------------------------------------- */

static float att_roll = 0.0f, att_pitch = 0.0f, att_yaw = 0.0f;
static uint32_t att_last_us = 0;
static bool att_init = false;

static void att_update(float ax, float ay, float az, float gx, float gy, float gz)
{
    uint32_t now = micros();
    float dt = (att_last_us == 0) ? 0.01f : (float)(now - att_last_us) / 1000000.0f;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;
    att_last_us = now;

    /* Roll / pitch from the gravity vector (valid in non-accelerating flight). */
    float acc_roll  = atan2f(ay, az) * RAD_TO_DEG;
    float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    if (!att_init)
    {
        att_roll  = acc_roll;
        att_pitch = acc_pitch;
        att_yaw   = 0.0f;
        att_init  = true;
        return;
    }

    /* Gyro body rates -> Euler angle rates (rad/s). */
    float p = gx * DEG_TO_RAD;
    float q = gy * DEG_TO_RAD;
    float r = gz * DEG_TO_RAD;

    float sp = sinf(att_roll * DEG_TO_RAD);
    float cp = cosf(att_roll * DEG_TO_RAD);
    float tt = tanf(att_pitch * DEG_TO_RAD);
    float ct = cosf(att_pitch * DEG_TO_RAD);
    if (fabsf(ct) < 0.1f) ct = (ct < 0.0f) ? -0.1f : 0.1f;

    float phi_dot   = p + sp * tt * q + cp * tt * r;
    float theta_dot = cp * q - sp * r;
    float psi_dot   = (sp / ct) * q + (cp / ct) * r;

    /* Complementary filter: blend gyro integration (fast, drifts) with the
       accel reference (noisy, no drift). alpha = 0.98 -> ~0.5 s time constant. */
    const float alpha = 0.98f;
    att_roll  = alpha * (att_roll  + phi_dot   * dt * RAD_TO_DEG) + (1.0f - alpha) * acc_roll;
    att_pitch = alpha * (att_pitch + theta_dot * dt * RAD_TO_DEG) + (1.0f - alpha) * acc_pitch;
    att_yaw  += psi_dot * dt * RAD_TO_DEG;
}

/* ------------------------------------------------------------------------- */
/* uBlox GNSS (USART1, NMEA)                                                 */
/* ------------------------------------------------------------------------- */

#define GPS_BAUD 9600

static const uint32_t GPS_BAUDS[] = { 9600, 38400, 115200, 4800, 57600, 19200, 230400, 460800, 921600 };
static uint8_t  gps_baud_idx   = 0;
static uint32_t gps_last_switch = 0;
static uint32_t gps_rx_bytes   = 0;
static bool     gps_locked     = false;
static bool     gps_nmea_valid = false;
static uint8_t  gps_first[160];
static uint16_t gps_first_len  = 0;

/* --- UBX (u-blox binary) helpers for active probing ---------------------- */

static void ubx_send(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    uint8_t ck_a = 0, ck_b = 0;
    Serial1.write(0xB5);
    Serial1.write(0x62);
    Serial1.write(cls);  ck_a += cls;              ck_b += ck_a;
    Serial1.write(id);   ck_a += id;               ck_b += ck_a;
    Serial1.write(len & 0xFF);        ck_a += (uint8_t)(len & 0xFF);         ck_b += ck_a;
    Serial1.write((len >> 8) & 0xFF); ck_a += (uint8_t)((len >> 8) & 0xFF);  ck_b += ck_a;
    for (uint16_t i = 0; i < len; i++)
    {
        Serial1.write(payload[i]);
        ck_a += payload[i];
        ck_b += ck_a;
    }
    Serial1.write(ck_a);
    Serial1.write(ck_b);
}

/* UBX-MON-VER poll: any u-blox module answers regardless of output config. */
static void ubx_poll_monver(void)
{
    ubx_send(0x0A, 0x04, NULL, 0);
}

/* UBX-CFG-PRT: configure UART1 -> NMEA + UBX out at 9600 8N1. */
static void ubx_enable_nmea(void)
{
    static const uint8_t prt[20] = {
        0x01, 0x00, 0x00, 0x00,       /* portID = UART1, reserved, txReady */
        0xD0, 0x08, 0x00, 0x00,       /* mode: 8N1 */
        0x80, 0x25, 0x00, 0x00,       /* baudRate: 9600 */
        0x03, 0x00,                   /* inProtoMask: UBX | NMEA */
        0x03, 0x00,                   /* outProtoMask: UBX | NMEA */
        0x00, 0x00, 0x00, 0x00        /* flags, reserved */
    };
    ubx_send(0x06, 0x00, prt, sizeof(prt));
}

/* Count signal edges on a pin over a window (microseconds). */
static uint32_t pin_edge_count(int pin, uint32_t window_us)
{
    uint32_t edges = 0;
    int last = digitalRead(pin);
    uint32_t t0 = micros();
    while ((micros() - t0) < window_us)
    {
        int cur = digitalRead(pin);
        if (cur != last)
        {
            edges++;
            last = cur;
        }
    }
    return edges;
}

/* Count edges on two pins concurrently over a window (microseconds). */
static void pin_edge_count2(int p1, int p2, uint32_t window_us,
                            uint32_t *e1, uint32_t *e2)
{
    uint32_t n1 = 0, n2 = 0;
    int l1 = digitalRead(p1), l2 = digitalRead(p2);
    uint32_t t0 = micros();
    while ((micros() - t0) < window_us)
    {
        int c1 = digitalRead(p1);
        if (c1 != l1) { n1++; l1 = c1; }
        int c2 = digitalRead(p2);
        if (c2 != l2) { n2++; l2 = c2; }
    }
    *e1 = n1; *e2 = n2;
}

/* Drain USART1 RX into the boot-burst capture buffer. */
static void gps_drain(void)
{
    while (Serial1.available())
    {
        uint8_t c = (uint8_t)Serial1.read();
        gps_rx_bytes++;
        if (gps_first_len < sizeof(gps_first)) gps_first[gps_first_len++] = c;
    }
}

/* Start USART1 with the correct (crossed) uBlox wiring:
   PA9 = TX (MCU -> uBlox RX), PA10 = RX (uBlox TX -> MCU). */
static void gps_uart_begin(uint32_t baud)
{
    Serial1.end();
    Serial1.setTx(PA9);
    Serial1.setRx(PA10);
    Serial1.begin(baud);
}

static char gps_line[128];
static uint8_t gps_len = 0;
static char gps_last_nmea[96];

/* Split a comma-separated NMEA field list, preserving empty fields. */
static int gps_split(char *s, char *f[], int max)
{
    int n = 0;
    f[n++] = s;
    for (char *p = s; *p && n < max; p++)
    {
        if (*p == ',')
        {
            *p = '\0';
            f[n++] = p + 1;
        }
    }
    return n;
}

/* Validate the trailing *XX checksum of any NMEA sentence. */
static bool gps_valid_nmea(const char *line)
{
    int len = (int)strlen(line);
    if (line[0] != '$' || len < 8 || line[len - 3] != '*') return false;
    uint8_t cs = 0;
    for (int i = 1; i < len - 3; i++) cs ^= (uint8_t)line[i];
    uint8_t cs_hex = (uint8_t)strtol(line + len - 2, NULL, 16);
    return cs == cs_hex;
}

static bool gps_process_line(char *line)
{
    int len = (int)strlen(line);

    /* Match NMEA GGA sentences ($GPGGA / $GNGGA / $GLGGA / ...). */
    if (line[0] != '$' || line[3] != 'G' || line[4] != 'G' || line[5] != 'A')
        return false;

    /* Validate the trailing *XX checksum. */
    if (len < 10 || line[len - 3] != '*') return false;
    uint8_t cs = 0;
    for (int i = 1; i < len - 3; i++) cs ^= (uint8_t)line[i];
    uint8_t cs_hex = (uint8_t)strtol(line + len - 2, NULL, 16);
    if (cs != cs_hex) return false;

    line[len - 3] = '\0';            /* drop checksum before splitting */

    char *f[15];
    int n = gps_split(line, f, 15);

    /* Need fields through altitude unit (index 10). */
    if (n < 11) return false;
    if (f[2][0] == '\0' || f[4][0] == '\0') return false;

    float lat_raw = atof(f[2]);
    float lat = (int)(lat_raw / 100.0f) + (lat_raw - (int)(lat_raw / 100.0f) * 100.0f) / 60.0f;
    if (f[3][0] == 'S') lat = -lat;

    float lon_raw = atof(f[4]);
    float lon = (int)(lon_raw / 100.0f) + (lon_raw - (int)(lon_raw / 100.0f) * 100.0f) / 60.0f;
    if (f[5][0] == 'W') lon = -lon;

    float gps_alt = atof(f[9]);
    int sats = atoi(f[7]);
    int fix  = atoi(f[6]);

    SerialUSB.print(F("GPS,"));
    SerialUSB.print(lat, 6);
    SerialUSB.print(',');
    SerialUSB.print(lon, 6);
    SerialUSB.print(',');
    SerialUSB.print(gps_alt, 1);
    SerialUSB.print(',');
    SerialUSB.print(sats);
    SerialUSB.print(',');
    SerialUSB.println(fix);
    return true;
}

/* ------------------------------------------------------------------------- */
/* Data logging (SD card over SDIO, FatFs)                                   */
/* ------------------------------------------------------------------------- */

static float g_ax = 0, g_ay = 0, g_az = 0, g_gx = 0, g_gy = 0, g_gz = 0;
static float g_press = 0, g_temp = 0, g_alt = 0;

static File sd_file;
static bool sd_ok = false;
static char sd_log_name[16];

/* ------------------------------------------------------------------------- */
/* setup / loop                                                              */
/* ------------------------------------------------------------------------- */

void setup(void)
{
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

    als_begin();

    SerialUSB.begin();
    gps_uart_begin(GPS_BAUD);

    /* Wait (bounded) for the USB host, draining uBlox boot burst meanwhile. */
    for (int i = 0; i < 300 && !SerialUSB; i++) { gps_drain(); delay(10); }
    delay(100);
    gps_drain();
    SerialUSB.println(F("boot: BMP581 + BMI323 + uBlox + SD streamer"));

    /* One-time I2C bus scan + ALS presence report. */
    i2c_scan();
    SerialUSB.print(F("ALS_STATUS,"));
    SerialUSB.println(als_present ? 1 : 0);
    als_debug_probe();
    gps_drain();

    /* --- SD card (SDIO 4-bit) + log file -------------------------------- */
    sd_ok = SD.begin();
    if (sd_ok)
    {
        int n = 0;
        do { snprintf(sd_log_name, sizeof(sd_log_name), "FLT%05d.CSV", n++); }
        while (SD.exists(sd_log_name));

        sd_file = SD.open(sd_log_name, FILE_WRITE);
        if (sd_file)
        {
            sd_file.println(F("t_ms,ax,ay,az,gx,gy,gz,roll,pitch,yaw,press_hPa,temp_c,alt_m"));
            sd_file.flush();
            SerialUSB.print(F("SD_STATUS,1,"));
            SerialUSB.println(sd_log_name);
        }
        else
        {
            sd_ok = false;
            SerialUSB.println(F("SD_STATUS,0,open_failed"));
        }
    }
    else
    {
        SerialUSB.println(F("SD_STATUS,0,begin_failed"));
    }
}

void loop(void)
{
    /* --- uBlox GNSS: read NMEA over USART1 ------------------------------- */
    while (Serial1.available())
    {
        char c = (char)Serial1.read();
        gps_rx_bytes++;
        if (gps_first_len < sizeof(gps_first)) gps_first[gps_first_len++] = (uint8_t)c;
        if (c == '\n')
        {
            gps_line[gps_len] = '\0';
            if (gps_valid_nmea(gps_line))
            {
                gps_nmea_valid = true;
                strncpy(gps_last_nmea, gps_line, sizeof(gps_last_nmea) - 1);
                gps_last_nmea[sizeof(gps_last_nmea) - 1] = '\0';
            }
            if (gps_process_line(gps_line)) gps_locked = true;
            gps_len = 0;
        }
        else if (c != '\r' && gps_len < (sizeof(gps_line) - 1))
        {
            gps_line[gps_len++] = c;
        }
    }

    /* Baud auto-detect: cycle until we see a valid NMEA sentence. */
    if (!gps_nmea_valid && (millis() - gps_last_switch) >= 2000)
    {
        gps_last_switch = millis();
        gps_baud_idx = (uint8_t)((gps_baud_idx + 1) % (sizeof(GPS_BAUDS) / sizeof(GPS_BAUDS[0])));
        gps_uart_begin(GPS_BAUDS[gps_baud_idx]);
    }

    /* Active u-blox probe: poll version + enable NMEA every ~2 s. */
    static uint32_t last_ubx = 0;
    if ((millis() - last_ubx) >= 2000)
    {
        last_ubx = millis();
        ubx_poll_monver();
        ubx_enable_nmea();
    }

    /* Wiring diagnostic: listen on PA9 (swap test) + PA10 (RX-present test)
       every ~4 s. Edges > ~5 mean data traffic on that pin. */
    static uint32_t last_pa = 0;
    if ((millis() - last_pa) >= 4000)
    {
        last_pa = millis();
        uint32_t cur_baud = GPS_BAUDS[gps_baud_idx];
        Serial1.end();
        pinMode(PA9, INPUT_PULLUP);
        uint32_t e9 = pin_edge_count(PA9, 50000);    /* 50 ms window */
        int l9 = digitalRead(PA9);
        pinMode(PA10, INPUT_PULLUP);
        uint32_t e10 = pin_edge_count(PA10, 50000);  /* 50 ms window */
        int l10 = digitalRead(PA10);
        gps_uart_begin(cur_baud);
        SerialUSB.print(F("PA_EDGES,"));
        SerialUSB.print(e9);
        SerialUSB.print(',');
        SerialUSB.print(l9);
        SerialUSB.print(',');
        SerialUSB.print(e10);
        SerialUSB.print(',');
        SerialUSB.println(l10);
    }

    /* Debug: report rx byte count / baud / lock state every 2 s. */
    static uint32_t last_gps_dbg = 0;
    if ((millis() - last_gps_dbg) >= 2000)
    {
        last_gps_dbg = millis();
        SerialUSB.print(F("GPS_DBG,"));
        SerialUSB.print(gps_rx_bytes);
        SerialUSB.print(',');
        SerialUSB.print(GPS_BAUDS[gps_baud_idx]);
        SerialUSB.print(',');
        SerialUSB.println(gps_locked ? 1 : 0);
    }

    /* Show captured uBlox boot bytes (hex) every 3 s. */
    static uint32_t last_gps_first = 0;
    if ((millis() - last_gps_first) >= 3000)
    {
        last_gps_first = millis();
        SerialUSB.print(F("GPS_FIRST,"));
        SerialUSB.print(gps_first_len);
        SerialUSB.print(',');
        for (uint16_t i = 0; i < gps_first_len && i < 64; i++)
        {
            if (gps_first[i] < 0x10) SerialUSB.print('0');
            SerialUSB.print(gps_first[i], HEX);
        }
        SerialUSB.println();
    }

    /* Echo the most recent valid NMEA sentence (1 Hz). */
    static uint32_t last_nmea = 0;
    if (gps_nmea_valid && (millis() - last_nmea) >= 1000)
    {
        last_nmea = millis();
        SerialUSB.print(F("GPS_RAW,"));
        SerialUSB.println(gps_last_nmea);
    }

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

            att_update(ax, ay, az, gx, gy, gz);
            g_ax = ax; g_ay = ay; g_az = az;
            g_gx = gx; g_gy = gy; g_gz = gz;

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

    /* --- Attitude output at ~20 Hz --------------------------------------- */
    static uint32_t last_att_out = 0;
    if (bmi_ready && (millis() - last_att_out) >= 50)
    {
        last_att_out = millis();
        SerialUSB.print(F("ATT,"));
        SerialUSB.print(att_roll, 1);
        SerialUSB.print(',');
        SerialUSB.print(att_pitch, 1);
        SerialUSB.print(',');
        SerialUSB.println(att_yaw, 1);
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

                g_press = p / 100.0f;
                g_temp  = t;
                g_alt   = alt;

                SerialUSB.print(F("BMP,"));
                SerialUSB.print(p / 100.0f, 3);
                SerialUSB.print(',');
                SerialUSB.print(t, 2);
                SerialUSB.print(',');
                SerialUSB.println(alt, 2);
            }
        }
    }

    /* --- ALS31300 reading at ~10 Hz (only if present) -------------------- */
    static uint32_t last_als = 0;
    if (als_present && (millis() - last_als) >= 100)
    {
        last_als = millis();
        int16_t ax_, ay_, az_, at_;
        uint8_t raw[8];
        if (als_get(&ax_, &ay_, &az_, &at_, raw))
        {
            float temp_c = 302.0f * ((float)at_ - 1708.0f) / 4096.0f;
            SerialUSB.print(F("ALS,"));
            SerialUSB.print(ax_);
            SerialUSB.print(',');
            SerialUSB.print(ay_);
            SerialUSB.print(',');
            SerialUSB.print(az_);
            SerialUSB.print(',');
            SerialUSB.println(temp_c, 1);
        }
    }

    /* --- ALS31300 raw bytes at ~2 Hz (only if present) -------------------- */
    static uint32_t last_als_raw = 0;
    if (als_present && (millis() - last_als_raw) >= 500)
    {
        last_als_raw = millis();
        uint8_t raw[8];
        if (als_read(0x28, raw, 8))
        {
            SerialUSB.print(F("ALS_RAW,"));
            for (int i = 0; i < 8; i++)
            {
                if (raw[i] < 0x10) SerialUSB.print('0');
                SerialUSB.print(raw[i], HEX);
                if (i < 7) SerialUSB.print(',');
            }
            SerialUSB.println();
        }
    }

    /* --- SD data log at ~50 Hz ------------------------------------------- */
    static uint32_t last_sd_log = 0;
    if (sd_ok && bmi_ready && (millis() - last_sd_log) >= 20)
    {
        last_sd_log = millis();

        sd_file.print(millis());
        sd_file.print(',');
        sd_file.print(g_ax, 4); sd_file.print(',');
        sd_file.print(g_ay, 4); sd_file.print(',');
        sd_file.print(g_az, 4); sd_file.print(',');
        sd_file.print(g_gx, 2); sd_file.print(',');
        sd_file.print(g_gy, 2); sd_file.print(',');
        sd_file.print(g_gz, 2); sd_file.print(',');
        sd_file.print(att_roll, 1); sd_file.print(',');
        sd_file.print(att_pitch, 1); sd_file.print(',');
        sd_file.print(att_yaw, 1); sd_file.print(',');
        sd_file.print(g_press, 3); sd_file.print(',');
        sd_file.print(g_temp, 2); sd_file.print(',');
        sd_file.println(g_alt, 2);

        static uint32_t last_sd_sync = 0;
        if ((millis() - last_sd_sync) >= 1000)
        {
            last_sd_sync = millis();
            sd_file.flush();
        }
    }

    /* --- SD status debug every 5 s --------------------------------------- */
    static uint32_t last_sd_dbg = 0;
    if ((millis() - last_sd_dbg) >= 5000)
    {
        last_sd_dbg = millis();
        SerialUSB.print(F("SD_DBG,"));
        SerialUSB.print(sd_ok ? 1 : 0);
        SerialUSB.print(F(","));
        SerialUSB.print(sd_log_name);
        SerialUSB.println();
    }
}
