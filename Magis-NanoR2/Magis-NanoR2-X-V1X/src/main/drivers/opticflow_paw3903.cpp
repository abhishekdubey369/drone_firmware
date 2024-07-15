/*
 * This file is part of Magis.
 *
 * Cleanflight and Magis are free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight and Magis are distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */
#define PAW_SPI_MODE

#ifndef PAW_SPI_MODE
	#define CXOF_HEADER         (uint8_t)0xFE
	#define CXOF_FOOTER         (uint8_t)0xAA
	#define CXOF_FRAME_LENGTH               9
	#define CXOF_PIXEL_SCALING1      (1.76e-3)
	#define CXOF_PIXEL_SCALING        0.00745
	#define CXOF_TIMEOUT_SEC             0.3f
#endif

#define OPTICFLOW_UPDATE_FREQUENCY	40000	//40ms

#define DISABLE_SPI       GPIO_SetBits(GPIOB,   GPIO_Pin_12)
#define ENABLE_SPI        GPIO_ResetBits(GPIOB, GPIO_Pin_12)

#define PRODUCT_ID_3903		0x49



#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/light_led.h"

#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/serial.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/flash_m25p16.h"
#include "drivers/flash.h"
#include "drivers/sc18is602b.h"

#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/sonar.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/display.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"

#include "rx/rx.h"
#include "rx/msp.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/gtune.h"
#include "flight/navigation.h"
#include "flight/filter.h"
#include "flight/acrobats.h"
#include "flight/posEstimate.h"
#include "flight/posControl.h"
#include "flight/opticflow.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "API/Utils.h"
#include "API/Peripheral.h"

#include "opticflow_paw3903.h"


int16_t flowScalerX = 0;
int16_t flowScalerY = 0;

uint32_t last_frame_us;             // system time of last message from flow sensor
uint8_t buf[13];                    // buff of characters received from flow sensor
uint8_t buf_len;                    // number of characters in buffer
float gyro_sum[2];                  // sum of gyro sensor values since last frame from flow sensor
uint16_t gyro_sum_count;            // number of gyro sensor values in sum
uint32_t last_opticflow_update_ms;


#define MAXIMUM_QUEUE_SIZE 3


uint8_t opticFlowAddress = 12;


void clear_buf(void);
void mode_0_init(void);
void mode_1_init(void);
void mode_2_init(void);

//Gyro Queue function
void addGyroXY(int16_t gyroX, int16_t gyroY);
void getFrontGyroXY(int16_t *gyroX, int16_t *gyroY);
bool isGyroXYQueueFull(void);

//De-rotation
void OpticFlow_Derotation(void);

static int16_t bufferX[MAXIMUM_QUEUE_SIZE], bufferY[MAXIMUM_QUEUE_SIZE];
static int16_t head = 0;
static int16_t rear = -1;
static int16_t itemCount = 0;


//AdityaLakshmi

// Optic flow sensor data check
//bool OpticFlowDataNewflag = false;


// Optic flow sensor data
uint8_t motion_reg  = 0;					  // motion register
uint8_t observation = 0;					  // observation
uint8_t surface_quality = 0;    			  // Surface Quality
uint16_t shutter = 0;	                      // Shutter
int16_t del_X = 0;  						  // del X after the value was swapped properly
int16_t del_Y = 0;                            // del Y after the value was swapped properly
int16_t del_X_derotated;					  // De-rotated Del X
int16_t del_Y_derotated;					  // De-rotated Del Y

uint8_t imu_tick = 0;						  // Tracking gyro
uint8_t OpticFlow_Mode = 0;

#ifndef PAW_SPI_MODE
SC18IS602B spiBridge;
uint8_t spiOpticFlowData[] = {(0x16 & ~0x80u),0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
bool ok=false;
#endif



bool initOpticFlow()
{
	Monitor.println("initOpticFlow");
uint8_t product_id;

	//Startup sequence
    GPIO.init(Pin14, OUTPUT);
    GPIO.write(Pin14, STATE_HIGH);

    delay(40);

    GPIO.write(Pin14, STATE_HIGH);
    delay(2);
    GPIO.write(Pin14, STATE_LOW);
    delay(2);
    GPIO.write(Pin14, STATE_HIGH);
    delay(2);

    //Initialisation
    opticFlowAddress = SPI.read(0x00);
    SPI.write(0x3A, 0x5A);
    delay(5);
    SPI.read(0x02);
    SPI.read(0x03);
    SPI.read(0x04);
    SPI.read(0x05);
    SPI.read(0x06);
    delay(1);

    OpticFlow_Mode = 1; //Default mode is low light

    if(OpticFlow_Mode == 1)
    	mode_1_init();	//Low light
    else if(OpticFlow_Mode == 2)
    	mode_2_init();	//Ultra-low light
    else
    	mode_0_init();	//Bright light

    delay(100);

    product_id = SPI.read(0x00);
    Monitor.println("Product_ID: ", product_id);
	if(product_id==PRODUCT_ID_3903)
		return 0; 	//No issue
	else
		return 1;	//Report issue

}

#ifdef TEST_OPTIC_FLOW
bool PollOpticFlow(uint32_t cTime){

	//Check every 1ms
	static uint32_t nextUpdateAt;
	static uint8_t gyro_tracker = 0;

	uint8_t motion_pin;


	if ((int32_t) (cTime - nextUpdateAt) < 0)
			   return false;
	nextUpdateAt = cTime + 1000;

	//Store gyro data
	//If we have a new gyro data is available add to the queue
	if(gyro_tracker != imu_tick){
		gyro_tracker = imu_tick; 				//Update gyro_tick
		addGyroXY(gyroADC[0],gyroADC[1]);		// Add gyro to queue 0 = X, 1 = Y
	}

	//Main Code
	//Check the pin status
	//If 0, then write to motion register and blink led

	motion_pin = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15);

	if(motion_pin==0x00){//Check if motion pin is active

		//OpticFlowDataNewflag = true;

		clear_buf(); //Clear buffer

		//Get Optic Flow data
		ENABLE_SPI;
		delayMicroseconds(5);	//min 1.2 uS
		spiTransferByte(SPI2, 0x16);//spiTransferByte(SPI2, (0x16 & ~0x80u));
		delayMicroseconds(5); 	//min 2 uS
		spiTransfer(SPI2, buf, NULL, 12);
		delayMicroseconds(5);	//min ? uS
		DISABLE_SPI;
		delayMicroseconds(5);	//min ? uS


		// Optic flow sensor data assigned
		motion_reg  = buf[0];								  // motion register
		observation = buf[1];								  // observation
		surface_quality =  buf[6]; 							  // Surface Quality
		shutter     = ((uint16_t) buf[10] << 8) | buf[11];	  // Shutter
		del_X =  (int16_t)((uint16_t) buf[5] << 8) | buf[4];  // del X after the value was swapped properly
		del_Y =  (int16_t)((uint16_t) buf[3] << 8) | buf[2];  // del Y after the value was swapped properly


		/* Send to BlackBox */
		//debugOpticFlowVar = motion_reg;
		//debugOpticFlowVar1 = observation;

		//debugOpticFlowVar2 = del_X;
		//debugOpticFlowVar3 = del_Y;
		debugOpticFlowVar4 = surface_quality;
		debugOpticFlowVar5 = shutter;
		/**/

		if(isOpticFlowDataValid())
			{
				//Missing condition
				OpticFlow_Derotation();
				return true;

			}
		else
			return false;

	}else {
		//Motion pin zero
		motion_reg  = 0;
		return false;
	}
/**/
}

bool isOpticFlowDataValid()
{

	if(OpticFlow_Mode == 1)
	{
	//Mode 1 Conditions
		if (surface_quality > 70 && shutter < 8176)
					return true;
			else
					return false;
	}


}

// Check for New optic data
/*bool isOpticflowDataNew (void)
{
	return OpticFlowDataNewflag;
}*/


// Optic flow de-rotated using gyroscope angular rates
void OpticFlow_Derotation()
{
	int16_t hist_gyroX=0, hist_gyroY=0;

	if (isGyroXYQueueFull())
	{
		getFrontGyroXY(&hist_gyroX, &hist_gyroY);
	}
	else
	{
		hist_gyroX = 0;
		hist_gyroY = 0;
	}


	//Gyroscope conversion factor is 0.0042
	del_X_derotated = del_X + 42*hist_gyroY/10000;
	del_Y_derotated = del_Y - 42*hist_gyroX/10000;
}


#endif






//Old function to be deleted
/*
void updateOpticFlow(uint32_t cTime)
{
	static uint32_t nextUpdateAt;

	if ((int32_t) (cTime - nextUpdateAt) < 0)
	           return;
	nextUpdateAt = cTime + OPTICFLOW_UPDATE_FREQUENCY;

    	//Total delay (20uS) + data transfer time (120uS) ??

	ENABLE_SPI;
	delayMicroseconds(5);	//min 1.2 uS

	spiTransferByte(SPI2, 0x16);//spiTransferByte(SPI2, (0x16 & ~0x80u));

	delayMicroseconds(5); 	//min 2 uS

	spiTransfer(SPI2, buf, NULL, 12);

	delayMicroseconds(5);	//min ? uS

	DISABLE_SPI;
	delayMicroseconds(5);	//min ? uS

	debugOpticFlowVar = buf[0];						//Motion
	debugOpticFlowVar1 = buf[1];					//Observation
	debugOpticFlowVar2 = (int16_t)((uint16_t) buf[3] << 8) | buf[2];	//Del
	debugOpticFlowVar3 = (int16_t)((uint16_t) buf[5] << 8) | buf[4];
	debugOpticFlowVar4 = buf[6];
	debugOpticFlowVar5 = ((uint16_t) buf[10] << 8) | buf[11];


	flowRate[0] = (int16_t)((uint16_t) buf[3] << 8) | buf[2];  //
	flowRate[1] = (int16_t)((uint16_t) buf[5] << 8) | buf[4];
	bodyRate1[0] = (int16_t) buf[6];
	bodyRate1[1] = (int16_t)((uint16_t) buf[10] << 8) | buf[11];

	bodyRate2[0] = (int16_t) buf[7];




	//Filtering
	uint32_t this_frame_us = micros();
	last_opticflow_update_ms = millis();

	float dt = (this_frame_us - last_frame_us);
	last_frame_us = this_frame_us;

	flowRate[0] *= (0.051);
	flowRate[1] *= (0.051);

	gyro_sum[0] = gyroADC[0] * 0.00106;
	gyro_sum[1] = gyroADC[1] * 0.00106;


	//if (isGyroXYQueueFull()) {
	//	getFrontGyroXY(&hist_gyroX, &hist_gyroY);
	//} else {
	//	hist_gyroX = gyro_sum[0];
	//	hist_gyroY = gyro_sum[1];
	// }


	bodyRate[0] = hist_gyroX;
	bodyRate[1] = hist_gyroY;

	bodyRate[0] = gyro_sum[0];
	bodyRate[1] = gyro_sum[1];

}
*/

void mode_0_init()
{	//Bright mode

    SPI.write(0x7F, 0x00);
    SPI.write(0x55, 0x01);
    SPI.write(0x50, 0x07);
    SPI.write(0x7F, 0x0E);
    SPI.write(0x43, 0x10);
    SPI.write(0x48, 0x02);
    SPI.write(0x7F, 0x00);
    SPI.write(0x51, 0x7B);
    SPI.write(0x50, 0x00);
    SPI.write(0x55, 0x00);
    SPI.write(0x7F, 0x00);

    SPI.write(0x61, 0xAD);
    SPI.write(0x7F, 0x03);
    SPI.write(0x40, 0x00);
    SPI.write(0x7F, 0x05);
    SPI.write(0x41, 0xB3);
    SPI.write(0x43, 0xF1);
    SPI.write(0x45, 0x14);
    SPI.write(0x5F, 0x34);
    SPI.write(0x7B, 0x08);
    SPI.write(0x5E, 0x34);
    SPI.write(0x5B, 0x32);
    SPI.write(0x6D, 0x32);
    SPI.write(0x45, 0x17);
    SPI.write(0x70, 0xE5);
    SPI.write(0x71, 0xE5);
    SPI.write(0x7F, 0x06);
    SPI.write(0x44, 0x1B);
    SPI.write(0x40, 0xBF);
    SPI.write(0x4E, 0x3F);	//Resolution = 78 of 168
    SPI.write(0x7F, 0x08);
    SPI.write(0x66, 0x44);
    SPI.write(0x65, 0x20);
    SPI.write(0x6A, 0x3A);
    SPI.write(0x61, 0x05);
    SPI.write(0x62, 0x05);
    SPI.write(0x7F, 0x09);
    SPI.write(0x4F, 0xAF);
    SPI.write(0x48, 0x80);
    SPI.write(0x49, 0x80);
    SPI.write(0x57, 0x77);
    SPI.write(0x5F, 0x40);
    SPI.write(0x60, 0x78);
    SPI.write(0x61, 0x78);
    SPI.write(0x62, 0x08);
    SPI.write(0x63, 0x50);
    SPI.write(0x7F, 0x0A);
    SPI.write(0x45, 0x60);
    SPI.write(0x7F, 0x00);

    SPI.write(0x4D, 0x11);
    SPI.write(0x55, 0x80);
    SPI.write(0x74, 0x21);
    SPI.write(0x75, 0x1F);
    SPI.write(0x4A, 0x78);
    SPI.write(0x4B, 0x78);
    SPI.write(0x44, 0x08);
    SPI.write(0x45, 0x50);
    SPI.write(0x64, 0xFE);
    SPI.write(0x65, 0x1F);
    SPI.write(0x72, 0x0A);
    SPI.write(0x73, 0x00);
    SPI.write(0x7F, 014);
    SPI.write(0x44, 0x84);
    SPI.write(0x65, 0x47);
    SPI.write(0x66, 0x18);
    SPI.write(0x63, 0x70);
    SPI.write(0x6F, 0x2C);
    SPI.write(0x7F, 0x15);
    SPI.write(0x48, 0x48);
    SPI.write(0x7F, 0x07);
    SPI.write(0x41, 0x0D);
    SPI.write(0x43, 0x14);
    SPI.write(0x4B, 0x0E);
    SPI.write(0x45, 0x0F);
    SPI.write(0x44, 0x42);
    SPI.write(0x4C, 0x80);
    SPI.write(0x7F, 0x10);
    SPI.write(0x5B, 0x03);
    SPI.write(0x7F, 0x07);
    SPI.write(0x40, 0x41);

    delay(10);

    SPI.write(0x7F, 0x00);
    SPI.write(0x32, 0x00);
    SPI.write(0x7F, 0x07);
    SPI.write(0x40, 0x40);
    SPI.write(0x7F, 0x06);
    SPI.write(0x68, 0x70);

    SPI.write(0x69, 0x01);
    SPI.write(0x7F, 0x0D);
    SPI.write(0x48, 0xC0);
    SPI.write(0x6F, 0xD5);
    SPI.write(0x7F, 0x00);
    SPI.write(0x5B, 0xA0);
    SPI.write(0x4E, 0xA8);	//Resolution = 168 / 168
    SPI.write(0x5A, 0x50);
    SPI.write(0x40, 0x80);
    SPI.write(0x73, 0x1F);

    delay(10);

    SPI.write(0x73, 0x00);

}


void mode_1_init()
{	//Low-light mode
	Monitor.println("Mode 1 Init");
    SPI.write(0x7F, 0x00);
    SPI.write(0x55, 0x01);
    SPI.write(0x50, 0x07);
    SPI.write(0x7F, 0x0E);
    SPI.write(0x43, 0x10);
    SPI.write(0x48, 0x02);
    SPI.write(0x7F, 0x00);
    SPI.write(0x51, 0x7B);
    SPI.write(0x50, 0x00);
    SPI.write(0x55, 0x00);
    SPI.write(0x7F, 0x00);
    SPI.write(0x61, 0xAD);
    SPI.write(0x7F, 0x03);
    SPI.write(0x40, 0x00);
    SPI.write(0x7F, 0x05);
    SPI.write(0x41, 0xB3);
    SPI.write(0x43, 0xF1);
    SPI.write(0x45, 0x14);
    SPI.write(0x5F, 0x34);
    SPI.write(0x7B, 0x08);

    SPI.write(0x5E, 0x34);
    SPI.write(0x5B, 0x65);
    SPI.write(0x6D, 0x65);
    SPI.write(0x45, 0x17);
    SPI.write(0x70, 0xE5);
    SPI.write(0x71, 0xE5);
    SPI.write(0x7F, 0x06);
    SPI.write(0x44, 0x1B);
    SPI.write(0x40, 0xBF);
    SPI.write(0x4E, 0x3F);	//Resolution = 63 / 168
    SPI.write(0x7F, 0x08);
    SPI.write(0x66, 0x44);
    SPI.write(0x65, 0x20);
    SPI.write(0x6A, 0x3A);
    SPI.write(0x61, 0x05);
    SPI.write(0x62, 0x05);
    SPI.write(0x7F, 0x09);
    SPI.write(0x4F, 0xAF);
    SPI.write(0x48, 0x80);
    SPI.write(0x49, 0x80);
    SPI.write(0x57, 0x77);
    SPI.write(0x5F, 0x40);
    SPI.write(0x60, 0x78);
    SPI.write(0x61, 0x78);
    SPI.write(0x62, 0x08);
    SPI.write(0x63, 0x50);
    SPI.write(0x7F, 0x0A);
    SPI.write(0x45, 0x60);
    SPI.write(0x7F, 0x00);
    SPI.write(0x4D, 0x11);
    SPI.write(0x55, 0x80);
    SPI.write(0x74, 0x21);
    SPI.write(0x75, 0x1F);
    SPI.write(0x4A, 0x78);
    SPI.write(0x4B, 0x78);
    SPI.write(0x44, 0x08);
    SPI.write(0x45, 0x50);
    SPI.write(0x64, 0xFE);

    SPI.write(0x65, 0x1F);
    SPI.write(0X72, 0x0A);
    SPI.write(0x73, 0x00);
    SPI.write(0x7F, 0x14);
    SPI.write(0x44, 0x84);
    SPI.write(0x65, 0x67);
    SPI.write(0x66, 0x18);
    SPI.write(0x63, 0x70);
    SPI.write(0x6F, 0x2C);
    SPI.write(0x7F, 0x15);
    SPI.write(0x48, 0x48);
    SPI.write(0x75, 0x07);
    SPI.write(0x41, 0x0D);
    SPI.write(0x43, 0x14);
    SPI.write(0x4B, 0x0E);
    SPI.write(0x45, 0x0F);
    SPI.write(0x44, 0x42);
    SPI.write(0x4C, 0x80);
    SPI.write(0x7F, 0x10);
    SPI.write(0x5B, 0x03);
    SPI.write(0x7F, 0x07);
    SPI.write(0x40, 0x41);

    delay(10);

    SPI.write(0x7F, 0x00);
    SPI.write(0x32, 0x00);
    SPI.write(0x7F, 0x07);
    SPI.write(0x40, 0x40);
    SPI.write(0x7F, 0x06);
    SPI.write(0x68, 0x70);
    SPI.write(0x69, 0x01);
    SPI.write(0x7F, 0x0D);
    SPI.write(0x48, 0xC0);
    SPI.write(0x6F, 0xD5);
    SPI.write(0x7F, 0x00);
    SPI.write(0x5B, 0xA0);
    SPI.write(0x4E, 0xA8);		//Resolution = 168 / 168
    SPI.write(0x5A, 0x50);
    SPI.write(0x40, 0x80);

    SPI.write(0x73, 0x1F);

    delay(10);

    SPI.write(0x73, 0x00);

}


void mode_2_init()
{	//Super low light mode
    SPI.write(0x7F, 0x00);
    SPI.write(0x55, 0x01);
    SPI.write(0x50, 0x07);
    SPI.write(0x7F, 0x0E);
    SPI.write(0x43, 0x10);
    SPI.write(0x48, 0x04);
    SPI.write(0x7F, 0x00);
    SPI.write(0x51, 0x7B);
    SPI.write(0x50, 0x00);
    SPI.write(0x55, 0x00);
    SPI.write(0x7F, 0x00);
    SPI.write(0x61, 0xAD);
    SPI.write(0x7F, 0x03);
    SPI.write(0x40, 0x00);
    SPI.write(0x7F, 0x05);
    SPI.write(0x41, 0xB3);
    SPI.write(0x43, 0xF1);
    SPI.write(0x45, 0x14);
    SPI.write(0x5F, 0x34);
    SPI.write(0x7B, 0x08);
    SPI.write(0x5E, 0x34);
    SPI.write(0x5B, 0x32);
    SPI.write(0x6D, 0x32);
    SPI.write(0x45, 0x17);
    SPI.write(0x70, 0xE5);
    SPI.write(0x71, 0xE5);
    SPI.write(0x7F, 0x06);
    SPI.write(0x44, 0x1B);
    SPI.write(0x40, 0xBF);


    SPI.write(0x4E, 0x3F);
    SPI.write(0x7F, 0x08);
    SPI.write(0x66, 0x44);
    SPI.write(0x65, 0x20);
    SPI.write(0x6A, 0x3A);
    SPI.write(0x61, 0x05);
    SPI.write(0x62, 0x05);
    SPI.write(0x7F, 0x09);
    SPI.write(0x4F, 0xAF);
    SPI.write(0x48, 0x80);
    SPI.write(0x49, 0x80);
    SPI.write(0x57, 0x77);
    SPI.write(0x5F, 0x40);
    SPI.write(0x60, 0x78);
    SPI.write(0x61, 0x78);
    SPI.write(0x62, 0x08);
    SPI.write(0x63, 0x50);
    SPI.write(0x7F, 0x0A);
    SPI.write(0x45, 0x60);
    SPI.write(0x7F, 0x00);
    SPI.write(0x4D, 0x11);
    SPI.write(0x55, 0x80);
    SPI.write(0x74, 0x21);
    SPI.write(0x75, 0x1F);
    SPI.write(0x4A, 0x78);
    SPI.write(0x4B, 0x78);
    SPI.write(0x44, 0x08);
    SPI.write(0x45, 0x50);
    SPI.write(0x64, 0xCE);
    SPI.write(0x65, 0x0B);
    SPI.write(0X72, 0x0A);
    SPI.write(0x73, 0x00);
    SPI.write(0x7F, 0x14);
    SPI.write(0x44, 0x84);
    SPI.write(0x65, 0x67);
    SPI.write(0x66, 0x18);
    SPI.write(0x63, 0x70);
    SPI.write(0x6F, 0x2C);


    SPI.write(0x7F, 0x15);
    SPI.write(0x48, 0x48);
    SPI.write(0x75, 0x07);
    SPI.write(0x41, 0x0D);
    SPI.write(0x43, 0x14);
    SPI.write(0x4B, 0x0E);
    SPI.write(0x45, 0x0F);
    SPI.write(0x44, 0x42);
    SPI.write(0x4C, 0x80);
    SPI.write(0x7F, 0x10);
    SPI.write(0x5B, 0x02);
    SPI.write(0x7F, 0x07);
    SPI.write(0x40, 0x41);

    delay(25);

    SPI.write(0x7F, 0x00);
    SPI.write(0x32, 0x44);
    SPI.write(0x7F, 0x07);
    SPI.write(0x40, 0x40);
    SPI.write(0x7F, 0x06);
    SPI.write(0x68, 0x40);
    SPI.write(0x69, 0x02);
    SPI.write(0x7F, 0x0D);
    SPI.write(0x48, 0xC0);
    SPI.write(0x6F, 0xD5);
    SPI.write(0x7F, 0x00);
    SPI.write(0x5B, 0xA0);
    SPI.write(0x4E, 0xA8);
    SPI.write(0x5A, 0x50);
    SPI.write(0x40, 0x80);
    SPI.write(0x73, 0x0B);

    delay(25);

    SPI.write(0x73, 0x00);

}





void clear_buf(void){
	uint8_t index;
	for(index = 0; index < sizeof(buf); index++)
		buf[index]=0;
}


//Gyroscope queue functions
bool isGyroXYQueueFull(void)
{
    return itemCount == MAXIMUM_QUEUE_SIZE;
}

void getFrontGyroXY(int16_t *gyroX, int16_t *gyroY)
{
    *gyroX = bufferX[head];
    *gyroY = bufferY[head];
    head++;

    if (head == MAXIMUM_QUEUE_SIZE) {
        head = 0;
    }
    itemCount--;
}

void addGyroXY(int16_t gyroX, int16_t gyroY)
{
    if (itemCount < MAXIMUM_QUEUE_SIZE) {
        rear++;
        if (rear >= MAXIMUM_QUEUE_SIZE) {
            rear = 0;
        }
        bufferX[rear] = gyroX;
        bufferY[rear] = gyroY;
        itemCount++;

    } else {
        if (++rear == MAXIMUM_QUEUE_SIZE) {
            rear = 0;
            bufferX[rear] = gyroX;
            bufferY[rear] = gyroY;
            head++;

        } else {

            bufferX[rear] = gyroX;
            bufferY[rear] = gyroY;
            head++;
            if (head == MAXIMUM_QUEUE_SIZE) {
                head = 0;
            }
        }
    }
}



//	Bridge / UART based modesss
#ifndef PAW_SPI_MODE
void mode_0_init_bridge()
{

    spiBridge.write(0x7F, 0x00);
    spiBridge.write(0x55, 0x01);
    spiBridge.write(0x50, 0x07);
    spiBridge.write(0x7F, 0x0E);
    spiBridge.write(0x43, 0x10);
    spiBridge.write(0x48, 0x02);
    spiBridge.write(0x7F, 0x00);
    spiBridge.write(0x51, 0x7B);
    spiBridge.write(0x50, 0x00);
    spiBridge.write(0x55, 0x00);
    spiBridge.write(0x7F, 0x00);

    spiBridge.write(0x61, 0xAD);
    spiBridge.write(0x7F, 0x03);
    spiBridge.write(0x40, 0x00);
    spiBridge.write(0x7F, 0x05);
    spiBridge.write(0x41, 0xB3);
    spiBridge.write(0x43, 0xF1);
    spiBridge.write(0x45, 0x14);
    spiBridge.write(0x5F, 0x34);
    spiBridge.write(0x7B, 0x08);
    spiBridge.write(0x5E, 0x34);
    spiBridge.write(0x5B, 0x32);
    spiBridge.write(0x6D, 0x32);
    spiBridge.write(0x45, 0x17);
    spiBridge.write(0x70, 0xE5);
    spiBridge.write(0x71, 0xE5);
    spiBridge.write(0x7F, 0x06);
    spiBridge.write(0x44, 0x1B);
    spiBridge.write(0x40, 0xBF);
    spiBridge.write(0x4E, 0x3F);
    spiBridge.write(0x7F, 0x08);
    spiBridge.write(0x66, 0x44);
    spiBridge.write(0x65, 0x20);
    spiBridge.write(0x6A, 0x3A);
    spiBridge.write(0x61, 0x05);
    spiBridge.write(0x62, 0x05);
    spiBridge.write(0x7F, 0x09);
    spiBridge.write(0x4F, 0xAF);
    spiBridge.write(0x48, 0x80);
    spiBridge.write(0x49, 0x80);
    spiBridge.write(0x57, 0x77);
    spiBridge.write(0x5F, 0x40);
    spiBridge.write(0x60, 0x78);
    spiBridge.write(0x61, 0x78);
    spiBridge.write(0x62, 0x08);
    spiBridge.write(0x63, 0x50);
    spiBridge.write(0x7F, 0x0A);
    spiBridge.write(0x45, 0x60);
    spiBridge.write(0x7F, 0x00);

    spiBridge.write(0x4D, 0x11);
    spiBridge.write(0x55, 0x80);
    spiBridge.write(0x74, 0x21);
    spiBridge.write(0x75, 0x1F);
    spiBridge.write(0x4A, 0x78);
    spiBridge.write(0x4B, 0x78);
    spiBridge.write(0x44, 0x08);
    spiBridge.write(0x45, 0x50);
    spiBridge.write(0x64, 0xFE);
    spiBridge.write(0x65, 0x1F);
    spiBridge.write(0x72, 0x0A);
    spiBridge.write(0x73, 0x00);
    spiBridge.write(0x7F, 014);
    spiBridge.write(0x44, 0x84);
    spiBridge.write(0x65, 0x47);
    spiBridge.write(0x66, 0x18);
    spiBridge.write(0x63, 0x70);
    spiBridge.write(0x6F, 0x2C);
    spiBridge.write(0x7F, 0x15);
    spiBridge.write(0x48, 0x48);
    spiBridge.write(0x7F, 0x07);
    spiBridge.write(0x41, 0x0D);
    spiBridge.write(0x43, 0x14);
    spiBridge.write(0x4B, 0x0E);
    spiBridge.write(0x45, 0x0F);
    spiBridge.write(0x44, 0x42);
    spiBridge.write(0x4C, 0x80);
    spiBridge.write(0x7F, 0x10);
    spiBridge.write(0x5B, 0x03);
    spiBridge.write(0x7F, 0x07);
    spiBridge.write(0x40, 0x41);

    delay(10);

    spiBridge.write(0x7F, 0x00);
    spiBridge.write(0x32, 0x00);
    spiBridge.write(0x7F, 0x07);
    spiBridge.write(0x40, 0x40);
    spiBridge.write(0x7F, 0x06);
    spiBridge.write(0x68, 0x70);

    spiBridge.write(0x69, 0x01);
    spiBridge.write(0x7F, 0x0D);
    spiBridge.write(0x48, 0xC0);
    spiBridge.write(0x6F, 0xD5);
    spiBridge.write(0x7F, 0x00);
    spiBridge.write(0x5B, 0xA0);
    spiBridge.write(0x4E, 0xA8);
    spiBridge.write(0x5A, 0x50);
    spiBridge.write(0x40, 0x80);
    spiBridge.write(0x73, 0x1F);

    delay(10);

    spiBridge.write(0x73, 0x00);

}

void updateUARTOpticFlow()
{
	//UART based Optic flow
	//https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_OpticalFlow/AP_OpticalFlow_CXOF.cpp
//    if(!updateTimer.set(40, true))
//        return;
//

    // record gyro values as long as they are being used
    // the sanity check of dt below ensures old gyro values are not used
    if (gyro_sum_count < 1000) {

        gyro_sum[0] += (gyroADC[0] * 0.00106);
        gyro_sum[1] += (gyroADC[1] * 0.00106);
        gyro_sum_count++;
    }

    // sensor values
    int32_t x_sum = 0;
    int32_t y_sum = 0;
    uint16_t qual_sum = 0;
    uint16_t count = 0;

    while (UART.rxBytesWaiting(UART2)) {

        uint8_t c = UART.read8(UART2);

        if (buf_len == 0) {
            if (c == CXOF_HEADER) {
                buf[buf_len++] = c;
            }
        } else {
            // add character to buffer
            buf[buf_len++] = c;

            // if buffer has 9 items try to decode it
            if (buf_len >= CXOF_FRAME_LENGTH) {
                // check last character matches footer
                if (buf[buf_len - 1] != CXOF_FOOTER) {
                    buf_len = 0;
                    continue;
                }

                // decode package
                int16_t x_raw = (int16_t)((uint16_t) buf[3] << 8) | buf[2];
                int16_t y_raw = (int16_t)((uint16_t) buf[5] << 8) | buf[4];

                // add to sum of all readings from sensor this iteration
                count++;
                x_sum += x_raw;
                y_sum += y_raw;
                qual_sum += buf[7];

                // clear buffer
                buf_len = 0;
            }
        }

    }

    // return without updating state if no readings
    if (count == 0) {
        return;
    }

    // average surface quality scaled to be between 0 and 255
    surface_quality = (constrain(qual_sum / count, 64, 78) - 64) * 255 / 14;

    // calculate dt
    uint32_t this_frame_us = micros();
    last_opticflow_update_ms = millis();

    float dt = (this_frame_us - last_frame_us) * 1.0e-6;
    last_frame_us = this_frame_us;

    // sanity check dt
    if (is_positive(dt) && (dt < CXOF_TIMEOUT_SEC)) {
        // calculate flow values

        float flowScaleFactorX = 1.0f + 0.001f * flowScalerX;
        float flowScaleFactorY = 1.0f + 0.001f * flowScalerY;

        // copy flow rates to state structure
        flowRate[0] = ((float) x_sum / count) * flowScaleFactorX;

        flowRate[1] = (((float) y_sum / count) * flowScaleFactorY);

        flowRate[0] *= CXOF_PIXEL_SCALING / dt;

        flowRate[1] *= CXOF_PIXEL_SCALING / dt;


        if (isGyroXYQueueFull()) {
            getFrontGyroXY(&hist_gyroX, &hist_gyroY);
        } else {
            hist_gyroX = (gyro_sum[0] / gyro_sum_count);
            hist_gyroY = (gyro_sum[1] / gyro_sum_count);
        }

        bodyRate[0] = hist_gyroX;
        bodyRate[1] = hist_gyroY;

        addGyroXY(gyro_sum[0] / gyro_sum_count, gyro_sum[1] / gyro_sum_count);

    } else {
        // first frame received in some time so cannot calculate flow values
        flowRate[0] = 0;
        flowRate[1] = 0;

        bodyRate[0] = 0;
        bodyRate[1] = 0;

    }

    // reset gyro sum
    gyro_sum[0] = 0;
    gyro_sum[1] = 0;

    gyro_sum_count = 0;

}

bool initBRIDGEOpticFlow(void)
{
uint8_t product_id;


    spiBridge.init(0, 1, 1, 1);

    spiBridge.configureSPI(false, SC18IS601B_SPIMODE_3, SC18IS601B_SPICLK_1843_kHz);

//Startup sequence
    GPIO.init(Pin14, OUTPUT);
    GPIO.write(Pin14, STATE_HIGH);

    delay(40);

    GPIO.write(Pin14, STATE_HIGH);
    delay(2);
    GPIO.write(Pin14, STATE_LOW);
    delay(2);
    GPIO.write(Pin14, STATE_HIGH);
    delay(2);

    //SPI Bridge mode
    opticFlowAddress= spiBridge.read(0x00);
    spiBridge.write(0x3A, 0x5A);
    delay(5);
    spiBridge.read(0x02);
    spiBridge.read(0x03);
    spiBridge.read(0x04);
    spiBridge.read(0x05);
    spiBridge.read(0x06);
    delay(1);
    mode_0_init_bridge();

   // spiBridge.Write(0x5B, 0xC0);

    delay(100);




if(product_id==PRODUCT_ID_3903)
	return 0; 	//No issue
else
	return 1;	//Report issue

}

void updateBRIDGEOpticFlow(uint32_t cTime){

	static uint32_t nextUpdateAt;

	if ((int32_t) (cTime - nextUpdateAt) < 0)
			   return;
	nextUpdateAt = cTime + OPTICFLOW_UPDATE_FREQUENCY;

	ok=spiBridge.spiTransfer(0, spiOpticFlowData, sizeof(spiOpticFlowData), buf);

	if(ok)
	   bodyRate2[1]=444;
	else
	   bodyRate2[1]=111;

	flowRate[0] = (int16_t)((uint16_t) buf[4] << 8) | buf[3];
	flowRate[1] = (int16_t)((uint16_t) buf[6] << 8) | buf[5];
	bodyRate1[0] = (int16_t) buf[7];
	bodyRate1[1] = (int16_t)((uint16_t) buf[11] << 8) | buf[12];


	bodyRate2[0] = (int16_t) buf[8];


	uint32_t this_frame_us = micros();
	last_opticflow_update_ms = millis();

	float dt = (this_frame_us - last_frame_us);
	last_frame_us = this_frame_us;

	flowRate[0] *= (0.051);
	flowRate[1] *= (0.051);

	gyro_sum[0] = gyroADC[0] * 0.00106;
	gyro_sum[1] = gyroADC[1] * 0.00106;

	/*
	if (isGyroXYQueueFull()) {
		getFrontGyroXY(&hist_gyroX, &hist_gyroY);
	} else {
		hist_gyroX = gyro_sum[0];
		hist_gyroY = gyro_sum[1];
	 }
	 */

	bodyRate[0] = hist_gyroX;
	bodyRate[1] = hist_gyroY;

	bodyRate[0] = gyro_sum[0];
	bodyRate[1] = gyro_sum[1];

}

#endif









