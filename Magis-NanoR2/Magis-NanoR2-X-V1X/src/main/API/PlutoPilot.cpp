// Do not remove the include below
#include "PlutoPilot.h"
#include "User.h"
#include "Utils.h"
#include "Control.h"
#include "Estimate.h"
#include "Sensor.h"
#include "platform.h"
#include "Peripheral.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "drivers/system.h"

// #include "io/serial_msp.h"

// #include "API-Utils.h"
#include "io/serial.h"
// #include "io/serial_cli.h"
#include "io/gps.h"

serialPortConfig_t serialGpsPort={
    .identifier = SERIAL_PORT_USART2,
    .functionMask = FUNCTION_GPS,
    .msp_baudrateIndex = 115200,
    .gps_baudrateIndex = 9600
};

gpsConfig_t gpsConfig = {
        .provider = GPS_UBLOX,           // Set the GPS provider to UBLOX
        .sbasMode = SBAS_AUTO,           // Set the SBAS mode to AUTO
        .autoConfig = GPS_AUTOCONFIG_ON, // Enable GPS auto configuration
        .autoBaud = GPS_AUTOBAUD_ON      // Enable GPS auto baud
    };

    // Initialize serialConfig_t
    serialConfig_t serialConfig={
        .reboot_character = 'R',
        .portConfigs = serialGpsPort
    };

   void delay(unsigned int milliseconds) {
       unsigned int microsecondDelay = milliseconds * 1000; // Convert milliseconds to microseconds
       unsigned int currentTime = 0;
       unsigned int endTime = microsecondDelay * 100; // Assuming CPU frequency is 100 MHz

       // Loop until the desired delay is achieved
       while (currentTime < endTime) {
           ++currentTime;
       }
   }

char* uint32ToString(uint32_t value) {
    // Determine the maximum number of digits in a uint32_t value
    const int max_digits = 10; // Maximum digits in a 32-bit unsigned integer
    const int buffer_size = max_digits + 1; // +1 for the null terminator

    // Allocate a buffer to hold the string representation
    char* buffer = new char[buffer_size];

    // Null-terminate the buffer
    buffer[buffer_size - 1] = '\0';

    // Convert the value to a string (in reverse order)
    int index = buffer_size - 2; // Start filling from the second last position
    do {
        buffer[index--] = (value % 10) + '0';
        value /= 10;
    } while (value > 0);

    // Move the result to the beginning of the buffer
    return buffer + index + 1;
}

//The setup function is called once at PlutoX's hardware startup
void plutoInit()
{
//Add your hardware initialization code here
// gps_ublox.init(UART2 , BAUD_RATE_9600);
// gpsInit(8,GPS_UBLOX)
gpsInit(&serialConfig,&gpsConfig);

}


bool t;
uint32_t val;
char* value;

//The function is called once before plutoLoop() when you activate developer mode
void onLoopStart()
{
//Do your one time stuff here
// t = gps_ublox.rxBytesWaiting(UART2);
gpsThread();

}



//The loop function is called in an endless loop
void plutoLoop()
{
//Add your repeated code here
       // Generate and print a random number between 1 and 10
        // t = gps_ublox.rxBytesWaiting(UART2);
        // val = gps_ublox.read32(UART2);
        // // serialize8Debug(val);
        // serialize8Debug(val);

        gpsThread();

        /* code */


}



//The function is called once after plutoLoop() when you deactivate developer mode
void onLoopFinish()
{
//Do your cleanup stuff here
Command.land(105);

}
