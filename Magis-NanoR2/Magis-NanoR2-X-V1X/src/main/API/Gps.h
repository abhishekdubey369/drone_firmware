/*
 * This file is part of Magis.
 *
 * Magis is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Magis is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>


#include "Specifiers.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
    UART2, UART3
} GPS_Port_e;

typedef enum {
    BAUD_RATE_4800,
    BAUD_RATE_9600,
    BAUD_RATE_14400,
    BAUD_RATE_19200,
    BAUD_RATE_38400,
    BAUD_RATE_57600,
    BAUD_RATE_115200,
    BAUD_RATE_128000,
    BAUD_RATE_256000
} GPS_Baud_Rate_e;

class GPS_P {
public:

    void init(GPS_Port_e PORT, GPS_Baud_Rate_e BAUD);

    uint8_t read8(GPS_Port_e PORT);

    uint16_t read16(GPS_Port_e PORT);

    uint32_t read32(GPS_Port_e PORT);

    void write(GPS_Port_e PORT, uint8_t data);

    void write(GPS_Port_e PORT, const char *str);

    void write(GPS_Port_e PORT, uint8_t* data, uint16_t length);

    bool rxBytesWaiting(GPS_Port_e PORT);

    bool txBytesFree(GPS_Port_e PORT);

};

extern GPS_P GPS_UBLO;

#ifdef __cplusplus
}
#endif

