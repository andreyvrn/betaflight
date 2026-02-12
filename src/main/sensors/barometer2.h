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

#pragma once

#include "pg/pg.h"
#include "drivers/barometer/barometer.h"

typedef struct baro2Config_s {
    uint8_t baro2_busType;
    uint8_t baro2_i2c_device;
    uint8_t baro2_i2c_address;
    uint8_t baro2_spi_device;
    ioTag_t baro2_spi_csn;
    uint8_t baro2_hardware;
} baro2Config_t;

PG_DECLARE(baro2Config_t, baro2Config);

typedef struct baro2_s {
    baroDev_t dev;
    float altitude;
    int32_t temperature;
    int32_t pressure;
} baro2_t;

extern baro2_t baro2;

void baro2PreInit(void);
void baro2Init(void);
bool isBaro2Ready(void);
float getBaro2Altitude(void);
uint32_t baro2Update(timeUs_t currentTimeUs);
