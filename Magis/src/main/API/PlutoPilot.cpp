// Do not remove the include below
#include "PlutoPilot.h"
#include "Peripheral.h"
#include "io/gps.h"
#include "Utils.h"
#include "io/serial.h"
#include "config/config.h"
//#include <stdint.h>


//initial declaration

Monitor_P disp;

serialPortConfig_t gpsportconfig = {
		.identifier = SERIAL_PORT_USART2,
		.functionMask = FUNCTION_GPS,
		.msp_baudrateIndex = 115200,
		.gps_baudrateIndex = 9600
};


serialConfig_t gpsSerial = {
		.reboot_character = 'R',
		.portConfigs = {
				gpsportconfig,
				{},
				{},
				{}
		}
};

//resetSerialConfig(&gpsSerial);

gpsConfig_t gpsconfig = {
		.provider = GPS_UBLOX,
		.sbasMode = SBAS_AUTO,
		.autoConfig = GPS_AUTOCONFIG_ON,
	    .autoBaud = GPS_AUTOBAUD_ON
};

//The setup function is called once at Pluto's hardware startup
void plutoInit(void)
{
// Add your hardware initialization code here
//	gps_ublo.init(UART2,BAUD_RATE_9600);
	gpsInit(&gpsSerial,&gpsconfig);
}



//The function is called once before plutoLoop when you activate Developer Mode
void onLoopStart(void)
{
// do your one time stuffs here
//	gps_ublo.rxBytesWaiting(UART2);
//	disp.print("this");
//	gpsThread();

}





// The loop function is called in an endless loop
void plutoLoop(void)
{
}



//The function is called once after plutoLoop when you deactivate Developer Mode
void onLoopFinish(void)
{

// do your cleanup stuffs here


}




