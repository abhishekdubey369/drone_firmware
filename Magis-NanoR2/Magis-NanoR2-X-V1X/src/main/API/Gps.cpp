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



#include "Gps.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "platform.h"
#include "build_config.h"
#include "common/utils.h"
#include "common/atomic.h"

#include "drivers/nvic.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/pwm_output.h"
#include "drivers/timer.h"
#include "drivers/timer_stm32f30x.h"
#include "drivers/timer_impl.h"
#include "drivers/adc.h"
#include "drivers/adc_impl.h"

#include "sensors/sensors.h"

#include "io/serial.h"
#include "io/gps.h"

#include "config/config.h"
#include "config/runtime_config.h"




static serialPort_t* uart2;
static serialPort_t* uart3;


uint32_t getBaud(GPS_Baud_Rate_e BAUD)
{
    switch (BAUD) {

        case BAUD_RATE_4800:

        return 4800;

        break;

        case BAUD_RATE_9600:

        return 9600;

        break;

        case BAUD_RATE_14400:

        return 14400;

        break;

        case BAUD_RATE_19200:

        return 19200;

        break;

        case BAUD_RATE_38400:

        return 38400;

        break;

        case BAUD_RATE_57600:

        return 57600;

        break;

        case BAUD_RATE_115200:

        return 115200;

        break;

        case BAUD_RATE_128000:

        return 128000;

        break;

        case BAUD_RATE_256000:

        return 256000;

        break;
    }
}

void GPS_P::init(GPS_Port_e PORT, GPS_Baud_Rate_e BAUD)
{
        uart2 = openSerialPort(SERIAL_PORT_USART2, FUNCTION_GPS, NULL,getBaud(BAUD), MODE_RXTX, SERIAL_NOT_INVERTED);
}

uint8_t GPS_P::read8(GPS_Port_e PORT)
{
        return serialRead(uart2) & 0xff;
}

uint16_t GPS_P::read16(GPS_Port_e PORT)
{

    uint16_t t = read8(PORT);

    t += (uint16_t) read8(PORT) << 8;

    return t;
}

uint32_t GPS_P::read32(GPS_Port_e PORT)
{

    uint32_t t = read16(PORT);

    t += (uint32_t) read16(PORT) << 16;

    return t;

}

void GPS_P::write(GPS_Port_e PORT, uint8_t data)
{
        serialWrite(uart2, data);

}

void GPS_P::write(GPS_Port_e PORT, const char *str)
{
        serialPrint(uart2, str);


}

void GPS_P::write(GPS_Port_e PORT, uint8_t* data, uint16_t length)
{
        while (length--) {

            serialWrite(uart2, *data);
            data++;
        }

}

bool GPS_P::rxBytesWaiting(GPS_Port_e PORT)
{
        return serialRxBytesWaiting(uart2);
}

bool GPS_P::txBytesFree(GPS_Port_e PORT)
{
return serialTxBytesFree(uart2);
}

GPS_P GPS_UBLO;

