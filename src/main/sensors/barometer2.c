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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_BARO2

#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_spl06.h"
#include "drivers/bus.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "sensors/barometer.h"
#include "sensors/sensors.h"

#include "scheduler/scheduler.h"

#include "barometer2.h"

baro2_t baro2;

PG_REGISTER_WITH_RESET_FN(baro2Config_t, baro2Config, PG_BARO2_CONFIG, 1);

void pgResetFn_baro2Config(baro2Config_t *config)
{
    config->baro2_hardware = BARO_SPL06;
    config->baro2_busType = BUS_TYPE_I2C;
    config->baro2_i2c_device = I2C_DEV_TO_CFG(I2CDEV_2);
    config->baro2_i2c_address = 0;
    config->baro2_spi_device = SPI_DEV_TO_CFG(SPIINVALID);
    config->baro2_spi_csn = IO_TAG_NONE;
}

#define BARO2_CALIBRATION_CYCLES 100

static uint16_t baro2CalibrationCycles = 0;
static uint16_t baro2CalibrationCycleCount = 0;
static float baro2GroundAltitude = 0.0f;
static bool baro2Calibrated = false;
static bool baro2Ready = false;

void baro2PreInit(void)
{
#ifdef USE_SPI
    if (baro2Config()->baro2_busType == BUS_TYPE_SPI) {
        ioPreinitByTag(baro2Config()->baro2_spi_csn, IOCFG_IPU, PREINIT_PIN_STATE_HIGH);
    }
#endif
}

static bool baro2Detect(baroDev_t *baroDev)
{
    extDevice_t *dev = &baroDev->dev;

    switch (baro2Config()->baro2_busType) {
#ifdef USE_I2C
    case BUS_TYPE_I2C:
        i2cBusSetInstance(dev, baro2Config()->baro2_i2c_device);
        dev->busType_u.i2c.address = baro2Config()->baro2_i2c_address;
        break;
#endif
#ifdef USE_SPI
    case BUS_TYPE_SPI:
        if (!spiSetBusInstance(dev, baro2Config()->baro2_spi_device)) {
            return false;
        }
        dev->busType_u.spi.csnPin = IOGetByTag(baro2Config()->baro2_spi_csn);
        break;
#endif
    default:
        return false;
    }

    if (baro2Config()->baro2_hardware == BARO_SPL06) {
#ifdef USE_BARO_SPL06
        if (spl06Detect(baroDev)) {
            return true;
        }
#endif
    }

    return false;
}

void baro2Init(void)
{
    baro2Ready = baro2Detect(&baro2.dev);
    if (baro2Ready) {
        baro2CalibrationCycles = BARO2_CALIBRATION_CYCLES;
        baro2CalibrationCycleCount = 0;
        baro2GroundAltitude = 0.0f;
        baro2Calibrated = false;
    }
}

bool isBaro2Ready(void)
{
    return baro2Ready;
}

static float baro2PressureToAltitude(const float pressure)
{
    return (1.0f - powf(pressure / 101325.0f, 0.190295f)) * 4433000.0f;
}

static void baro2PerformCalibrationCycle(const float altitude)
{
    baro2GroundAltitude += altitude;
    baro2CalibrationCycleCount++;

    if (baro2CalibrationCycleCount >= baro2CalibrationCycles) {
        baro2GroundAltitude /= baro2CalibrationCycleCount;
        baro2Calibrated = true;
        baro2CalibrationCycleCount = 0;
    }
}

typedef enum {
    BARO2_STATE_TEMPERATURE_READ = 0,
    BARO2_STATE_TEMPERATURE_SAMPLE,
    BARO2_STATE_PRESSURE_START,
    BARO2_STATE_PRESSURE_READ,
    BARO2_STATE_PRESSURE_SAMPLE,
    BARO2_STATE_TEMPERATURE_START,
    BARO2_STATE_COUNT
} baro2State_e;

uint32_t baro2Update(timeUs_t currentTimeUs)
{
    static timeUs_t baro2StateDurationUs[BARO2_STATE_COUNT];
    static baro2State_e state = BARO2_STATE_PRESSURE_START;
    baro2State_e oldState = state;
    timeUs_t executeTimeUs;
    timeUs_t sleepTime = 1000;

    if (busBusy(&baro2.dev.dev, NULL)) {
        schedulerIgnoreTaskStateTime();
        return sleepTime;
    }

    switch (state) {
    default:
    case BARO2_STATE_TEMPERATURE_START:
        baro2.dev.start_ut(&baro2.dev);
        state = BARO2_STATE_TEMPERATURE_READ;
        sleepTime = baro2.dev.ut_delay;
        break;

    case BARO2_STATE_TEMPERATURE_READ:
        if (baro2.dev.read_ut(&baro2.dev)) {
            state = BARO2_STATE_TEMPERATURE_SAMPLE;
        } else {
            schedulerIgnoreTaskExecTime();
        }
        break;

    case BARO2_STATE_TEMPERATURE_SAMPLE:
        if (baro2.dev.get_ut(&baro2.dev)) {
            state = BARO2_STATE_PRESSURE_START;
        } else {
            schedulerIgnoreTaskExecTime();
        }
        break;

    case BARO2_STATE_PRESSURE_START:
        baro2.dev.start_up(&baro2.dev);
        state = BARO2_STATE_PRESSURE_READ;
        sleepTime = baro2.dev.up_delay;
        break;

    case BARO2_STATE_PRESSURE_READ:
        if (baro2.dev.read_up(&baro2.dev)) {
            state = BARO2_STATE_PRESSURE_SAMPLE;
        } else {
            schedulerIgnoreTaskExecTime();
        }
        break;

    case BARO2_STATE_PRESSURE_SAMPLE:
        if (!baro2.dev.get_up(&baro2.dev)) {
            schedulerIgnoreTaskExecTime();
            break;
        }

        baro2.dev.calculate(&baro2.pressure, &baro2.temperature);

        if (baro2.pressure > 0) {
            const float altitude = baro2PressureToAltitude(baro2.pressure);
            if (baro2Calibrated) {
                baro2.altitude = altitude - baro2GroundAltitude;
            } else {
                baro2PerformCalibrationCycle(altitude);
                baro2.altitude = 0.0f;
            }
        } else {
            if (!baro2Calibrated) {
                baro2.altitude = 0.0f;
            }
        }

        if (baro2.dev.combined_read) {
            state = BARO2_STATE_PRESSURE_START;
        } else {
            state = BARO2_STATE_TEMPERATURE_START;
        }
        break;
    }

    if (state != BARO2_STATE_PRESSURE_START) {
        schedulerIgnoreTaskExecRate();
    }

    executeTimeUs = micros() - currentTimeUs;

    if (executeTimeUs > baro2StateDurationUs[oldState]) {
        baro2StateDurationUs[oldState] = executeTimeUs;
    }

    schedulerSetNextStateTime(baro2StateDurationUs[state]);

    return sleepTime;
}

float getBaro2Altitude(void)
{
    return baro2.altitude;
}

#endif // USE_BARO2
