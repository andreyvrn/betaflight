/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// Goertek SPL06-001 barometric pressure sensor driver.
// Datasheet: https://datasheet.lcsc.com/lcsc/2101201914_Goertek-SPL06-001_C2684428.pdf

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_BARO) && defined(USE_BARO_SPL06)

#include "build/build_config.h"
#include "build/debug.h"

#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "barometer.h"
#include "barometer_spl06.h"

// I2C address
#define SPL06_I2C_ADDR                  0x76

// Register map
#define SPL06_REG_PSR_B2                0x00
#define SPL06_REG_PSR_B1                0x01
#define SPL06_REG_PSR_B0                0x02
#define SPL06_REG_TMP_B2                0x03
#define SPL06_REG_TMP_B1                0x04
#define SPL06_REG_TMP_B0                0x05
#define SPL06_REG_PRS_CFG               0x06
#define SPL06_REG_TMP_CFG               0x07
#define SPL06_REG_MEAS_CFG              0x08
#define SPL06_REG_CFG                   0x09
#define SPL06_REG_RESET                 0x0C
#define SPL06_REG_CHIP_ID               0x0D
#define SPL06_REG_COEF                  0x10    // 18 bytes: 0x10..0x21

// Chip ID
#define SPL06_CHIP_ID                   0x10

// Measurement mode: continuous pressure + temperature
#define SPL06_MEAS_CONTINUOUS_PT        0x07

// Pressure config: rate 32 Hz (bits [6:4] = 0b101), oversampling 16x (bits [3:0] = 0b0100)
#define SPL06_PRS_CFG_VALUE             0x54

// Temperature config: external sensor (bit 7), rate 32 Hz (bits [6:4] = 0b101), oversampling 1x (bits [3:0] = 0b000)
#define SPL06_TMP_CFG_VALUE             0xA0

// CFG register: P_SHIFT bit (required when pressure oversampling > 8x)
#define SPL06_CFG_P_SHIFT               0x04

// Soft reset command
#define SPL06_RESET_CMD                 0x89

// Scale factors for oversampling (from datasheet table 4)
#define SPL06_SCALE_FACTOR_P            253952.0f   // 16x pressure oversampling
#define SPL06_SCALE_FACTOR_T            524288.0f   // 1x temperature oversampling

// Calibration coefficients (datasheet section 8.11)
typedef struct {
    int16_t c0;     // 12-bit, signed
    int16_t c1;     // 12-bit, signed
    int32_t c00;    // 20-bit, signed
    int32_t c10;    // 20-bit, signed
    int16_t c01;    // 16-bit, signed
    int16_t c11;    // 16-bit, signed
    int16_t c20;    // 16-bit, signed
    int16_t c21;    // 16-bit, signed
    int16_t c30;    // 16-bit, signed
} spl06Calib_t;

static spl06Calib_t calib;
static int32_t spl06RawPressure;
static int32_t spl06RawTemperature;

static void spl06ReadCalibration(extDevice_t *dev)
{
    uint8_t buf[18];
    busReadRegisterBuffer(dev, SPL06_REG_COEF, buf, sizeof(buf));

    // c0: buf[0][7:0] buf[1][7:4] — 12-bit signed
    calib.c0 = ((int16_t)buf[0] << 4) | ((buf[1] >> 4) & 0x0F);
    if (calib.c0 & 0x0800) {
        calib.c0 |= 0xF000;
    }

    // c1: buf[1][3:0] buf[2][7:0] — 12-bit signed
    calib.c1 = ((int16_t)(buf[1] & 0x0F) << 8) | buf[2];
    if (calib.c1 & 0x0800) {
        calib.c1 |= 0xF000;
    }

    // c00: buf[3] buf[4] buf[5][7:4] — 20-bit signed
    calib.c00 = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | ((buf[5] >> 4) & 0x0F);
    if (calib.c00 & 0x080000) {
        calib.c00 |= 0xFFF00000;
    }

    // c10: buf[5][3:0] buf[6] buf[7] — 20-bit signed
    calib.c10 = ((int32_t)(buf[5] & 0x0F) << 16) | ((int32_t)buf[6] << 8) | buf[7];
    if (calib.c10 & 0x080000) {
        calib.c10 |= 0xFFF00000;
    }

    // c01..c30: 16-bit signed, big-endian pairs
    calib.c01 = ((int16_t)buf[8]  << 8) | buf[9];
    calib.c11 = ((int16_t)buf[10] << 8) | buf[11];
    calib.c20 = ((int16_t)buf[12] << 8) | buf[13];
    calib.c21 = ((int16_t)buf[14] << 8) | buf[15];
    calib.c30 = ((int16_t)buf[16] << 8) | buf[17];
}

// In continuous mode start functions are no-ops

static bool spl06StartUT(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

static bool spl06ReadUT(baroDev_t *baro)
{
    uint8_t buf[3];

    if (!busReadRegisterBuffer(&baro->dev, SPL06_REG_TMP_B2, buf, sizeof(buf))) {
        return false;
    }

    spl06RawTemperature = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    if (spl06RawTemperature & 0x800000) {
        spl06RawTemperature |= 0xFF000000;
    }

    return true;
}

static bool spl06GetUT(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

static bool spl06StartUP(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

static bool spl06ReadUP(baroDev_t *baro)
{
    uint8_t buf[3];

    if (!busReadRegisterBuffer(&baro->dev, SPL06_REG_PSR_B2, buf, sizeof(buf))) {
        return false;
    }

    spl06RawPressure = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    if (spl06RawPressure & 0x800000) {
        spl06RawPressure |= 0xFF000000;
    }

    return true;
}

static bool spl06GetUP(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

// Compensated pressure and temperature per datasheet section 4.9
static void spl06Calculate(int32_t *pressure, int32_t *temperature)
{
    const float fTsc = (float)spl06RawTemperature / SPL06_SCALE_FACTOR_T;
    const float fPsc = (float)spl06RawPressure / SPL06_SCALE_FACTOR_P;

    const float tempC = (float)calib.c0 * 0.5f + (float)calib.c1 * fTsc;

    if (temperature) {
        *temperature = (int32_t)(tempC * 100.0f);   // centidegrees
    }

    const float press = (float)calib.c00
        + fPsc * ((float)calib.c10 + fPsc * ((float)calib.c20 + fPsc * (float)calib.c30))
        + fTsc * (float)calib.c01
        + fTsc * fPsc * ((float)calib.c11 + fPsc * (float)calib.c21);

    if (pressure) {
        *pressure = (int32_t)press;   // Pa
    }
}

bool spl06Detect(baroDev_t *baro)
{
    extDevice_t *dev = &baro->dev;
    bool defaultAddressApplied = false;

    delay(20);

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        dev->busType_u.i2c.address = SPL06_I2C_ADDR;
        defaultAddressApplied = true;
    }

    uint8_t chipId = 0;
    busReadRegisterBuffer(dev, SPL06_REG_CHIP_ID, &chipId, 1);

    if (chipId != SPL06_CHIP_ID) {
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    busDeviceRegister(dev);

    // Soft reset and wait for completion
    busWriteRegister(dev, SPL06_REG_RESET, SPL06_RESET_CMD);
    delay(50);

    // Read calibration coefficients
    spl06ReadCalibration(dev);

    // Configure measurement parameters
    busWriteRegister(dev, SPL06_REG_PRS_CFG, SPL06_PRS_CFG_VALUE);
    busWriteRegister(dev, SPL06_REG_TMP_CFG, SPL06_TMP_CFG_VALUE);
    busWriteRegister(dev, SPL06_REG_CFG, SPL06_CFG_P_SHIFT);

    // Start continuous pressure + temperature measurement
    busWriteRegister(dev, SPL06_REG_MEAS_CFG, SPL06_MEAS_CONTINUOUS_PT);

    // Configure baroDev function pointers
    baro->combined_read = true;
    baro->ut_delay = 0;
    baro->up_delay = 32000;   // ~32ms at 32 Hz

    baro->start_ut = spl06StartUT;
    baro->read_ut = spl06ReadUT;
    baro->get_ut = spl06GetUT;

    baro->start_up = spl06StartUP;
    baro->read_up = spl06ReadUP;
    baro->get_up = spl06GetUP;

    baro->calculate = spl06Calculate;

    return true;
}

#endif // USE_BARO && USE_BARO_SPL06
