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
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/sensor.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "io/serial_msp.h"

#include "flight/pid.h"
#include "flight/imu.h"

#include "config/runtime_config.h"

#include "posEstimate.h"


#include "posControl.h"

//AdityaLakshmi
#include "flight/opticflow.h"
#include "../drivers/opticflow_paw3903.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/altitudehold.h"

#define corr_scale 512/100
#define corr_scale2 1096/100
#define MILLI_SEC 1000
#define SEC 1000000

void resetFilter(void);

static pidProfile_t *pidProfile; //PS2



int16_t posX = 0;
int16_t posY = 0;
int16_t posZ = 0;
int16_t deltaTime = 0;

int16_t debugAccelvar = 0;
int16_t debugAccelvar1 = 0;
int16_t debugAccelvar2 = 0;
int16_t debugAccelvar3= 0;

int8_t Quality=-1;
bool new_position=false;
int8_t localisationType = -1;

float inputX = 0.0f;
float inputY = 0.0f;
float dTime = 0.0f;
float inputZ = 0.0f; //

float inputPreX = 0.0f;
float inputPreY = 0.0f;

static int16_t first_read = 0;


float accel_EF[2];
float accel_EF_correction[2];
float est_velocity[2];
float accel_EF_prev[2];
float position_error[2];
float position_base[2];
float position_correction[2];
float est_position[2];
float velocity_increase[2];

uint8_t hist_xy_counter; // counter used to slow saving of position estimates for later comparison
float hist_position_baseX;
float hist_position_baseY;

float time_constant_xy = 1.5f;  // can be tuned
float k1;
float k2;
float k3;

int16_t PositionX = 0;
int16_t PositionY = 0;
//float PrevPositionX = 0;
//float PrevPositionY = 0;
int16_t VelocityX = 0;
int16_t VelocityY = 0;


#define BUFFER_XY_MAXIMUM 4 //can be tuned
#define INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS 10

float bufferX[BUFFER_XY_MAXIMUM], bufferY[BUFFER_XY_MAXIMUM];
int16_t head = 0;
int16_t rear = -1;
int16_t itemCount = 0;

//AdityaLakshmi
void updateFromOpticFlow(void);
bool isVelocityXYBaseQueueFull();
void getFrontVelocityXY(float *VelocityX, float *VelocityY);
void addVelocityXY(float VelocityX, float VelocityY);
//void UpdateAccelXY(void);

//Queue for Velocity queue
#define VELOCITY_QUEUE_SIZE 4			// 3/4
float Vel_buffX[VELOCITY_QUEUE_SIZE]; // #define
float Vel_buffY[VELOCITY_QUEUE_SIZE];
int16_t Vel_head = 0;
int16_t Vel_rear = -1;
int16_t Vel_itemCount = 0;


void resetPosition(void)
{       //Reset Pos and velocity
    est_position[0] = 0;
    est_position[1] = 0;
    est_velocity[0] = 0;
    est_velocity[1] = 0;
    resetPosIntegral();
}


void addHistPositionBaseEstXY(float positionX, float positionY){
    // determine position of new item
    rear = head + itemCount;
    if( rear >= BUFFER_XY_MAXIMUM ) {
        rear -= BUFFER_XY_MAXIMUM;
    }

    // add item to buffer

    bufferX[rear] = positionX;
    bufferY[rear] = positionY;

    // increment number of items
    if( itemCount < BUFFER_XY_MAXIMUM ) {
        itemCount++;
    }else{
        // no room for new items so drop oldest item
        head++;
        if( head >= BUFFER_XY_MAXIMUM ) {
            head = 0;
        }
    }
}

void getFrontHistPositionBaseEstXY(float *posbaseX, float *posbaseY)
{

	if(itemCount==0){
		*posbaseX=bufferX[head];
		*posbaseY=bufferY[head];
	return;
	}

	*posbaseX=bufferX[head];
	*posbaseY=bufferY[head];
	head++;

	if(head>=BUFFER_XY_MAXIMUM)
	{
		head=0;
	}
	itemCount--;
}



bool isPositionBaseXYQueueIsFull(void)
{
	return itemCount>=BUFFER_XY_MAXIMUM;
}



void checkPosition()
{

	switch(localisationType)
	{
		case LOC_UWB:
		// for uwb

		inputX = 1.0f*(float)posX;///10;	//centimeter
		inputY = -1.0f*(float)posY;///10;
		inputZ = 1.0f*(float)posZ;//10;
		break;

		case LOC_WHYCON:
		// for whycon
		inputX = -1.0f*(float)posY;///10;	//centimeter
		inputY = -1.0f*(float)posX;///10;
		break;

		case LOC_VICON:
		// for vicon
		inputX = 1.0f*(float)posX;///10;	//centimeter
		inputY = 1.0f*(float)posY;///10;
		inputZ = 1.0f*(float)posZ;//10;
		break;

		default:
		break;

	}

	//Discard the first 10 readings
	if(first_read <= 10) {
		setPos(inputX, inputY);
		first_read++;
	}


	if (isPositionBaseXYQueueIsFull())
		getFrontHistPositionBaseEstXY(&hist_position_baseX, &hist_position_baseY);
	else{
		hist_position_baseX = position_base[0];
		hist_position_baseY = position_base[1];
	}

	position_error[0] = inputX - (hist_position_baseX + position_correction[0]);
	position_error[1] = inputY - (hist_position_baseY + position_correction[1]);

}




/*
void PosXYEstimate(uint32_t currentTime)
{
    static uint32_t previousTime = 0, previousReadTime = 0;
    float dt = ((float)currentTime - (float)previousTime) / 1000000.0f;       //sec
    uint8_t i;


    if ((currentTime - previousTime) < 10*MILLI_SEC)	//10ms
        return;

    previousTime = currentTime;

	if(new_position){
		checkPosition();
		new_position = false;
		}
		else if((currentTime-previousReadTime > 3*SEC) && (!ARMING_FLAG(ARMED)))//Didn't receive data for 2 seconds reset integrator
		{
//			resetPosition(); Later uncomment for faster convergence
		}


    for (i = 0; i < 2; i++) {
        if (accSumCountXYZ) {
            accel_EF_prev[i] = accel_EF[i];
            accel_EF[i] = (float) accSum[i] / (float) accSumCountXYZ;
        } else {
        	accel_EF[i] = 0;
        }

        accel_EF[i] = accel_EF[i] * accVelScale;
        accel_EF[i] = constrainf(accel_EF[i], -800, 800);


        if (i == 1)
            imuResetAccelerationSum(0);

        accel_EF_correction[i] += position_error[i] * k3 * dt;
        est_velocity[i] += position_error[i] * k2 * dt;
        position_correction[i] += position_error[i] * k1 * dt;

        //Estimation
        accel_EF[i] += accel_EF_correction[i];
        velocity_increase[i] = (accel_EF[i]) * dt; // acc * dt
        position_base[i] += (est_velocity[i] + velocity_increase[i] * 0.5f)
                * dt; //S = S0 + u + (1/2) at^2

        //Updated pos and vel
        est_position[i] = position_base[i] + position_correction[i];
        est_velocity[i] += velocity_increase[i]; //v = u + at
    }



    // store 3rd order estimate (i.e. horizontal position) for future use at 10hz
    hist_xy_counter++;
    if (hist_xy_counter >= INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS) {

        hist_xy_counter = 0;
        addHistPositionBaseEstXY(position_base[0], position_base[1]);

    }

    PositionX = (int16_t) est_position[0];
    PositionY = (int16_t) est_position[1];
    VelocityX = (int16_t) est_velocity[0];
    VelocityY = (int16_t) est_velocity[1];

}
*/
void updatePosGains(void)
{
    //time_constant_xy=(float)pidProfile->I8[PIDNAVR]/10;
    if (time_constant_xy == 0.0f) {
        k1 = k2 = k3 = 0.0f;
    } else {
        k1 = 3.0f / time_constant_xy;
        k2 = 3.0f / (time_constant_xy * time_constant_xy);
        k3 = 1.0f / (time_constant_xy * time_constant_xy * time_constant_xy);
    }
}

void setPos(float newX, float newY)
{
    position_base[0] = newX;
    position_base[1] = newY;
    position_correction[0] = position_correction[1] = 0;
	est_position[0] = newX;
	est_position[1] = newY;

//    clearQueue();

//    hist_xy_counter = 0;
//    addHistPositionBaseEstXY(position_base[0], position_base[1]);
    imuResetAccelerationSum(0);
}

void configurePosHold2(pidProfile_t *initialPidProfile)
{
    pidProfile = initialPidProfile;
}

//AdityaLakshmi

// Accelerometer queue function
#ifdef OPTIC_FLOW_POSITION_ESTIMATION



bool isVelocityXYBaseQueueFull(void)
{
    return Vel_itemCount == VELOCITY_QUEUE_SIZE;
}

void getFrontVelocityXY(float *VelocityX, float *VelocityY)
{
    *VelocityX = Vel_buffX[Vel_head];
    *VelocityY = Vel_buffY[Vel_head];
    Vel_head++;

    if (Vel_head == VELOCITY_QUEUE_SIZE) {
        Vel_head = 0;
    }
    Vel_itemCount--;
}

void addVelocityXY(float VelocityX, float VelocityY)
{
    if (Vel_itemCount < VELOCITY_QUEUE_SIZE) {
        Vel_rear++;
        if (Vel_rear >= VELOCITY_QUEUE_SIZE) {
            Vel_rear = 0;
        }
        Vel_buffX[Vel_rear] = VelocityX;
        Vel_buffY[Vel_rear] = VelocityY;
        Vel_itemCount++;

    } else {
        if (++Vel_rear == VELOCITY_QUEUE_SIZE) {
            Vel_rear = 0;
            Vel_buffX[Vel_rear] = VelocityX;
            Vel_buffY[Vel_rear] = VelocityY;
            Vel_head++;

        } else {

        	Vel_buffX[Vel_rear] = VelocityX;
        	Vel_buffY[Vel_rear] = VelocityY;
        	Vel_head++;
            if (Vel_head == VELOCITY_QUEUE_SIZE) {
            	Vel_head = 0;
            }
        }
    }
}

#endif


//#ifdef OPTIC_FLOW_VALIDATION
//void checkOpticflow()
//{
	//if (isOpticflowdataValid() && ToF_Height >-10 )
	// && ABS(getEstVelocity()) < 20 // unsure
	//if (isOpticFlowDataValid() && ToF_Height >-10 )
	//{
		//OpticFlowValidationflag = true;
	//}
	//else {

		//OpticFlowValidationflag = false;
	//}
//}

//#endif


//Complementary filter used for estimate of velocity and position
#ifdef OPTIC_FLOW_POSITION_ESTIMATION

float VelocityXY_correction[2];  // Complementary Filter velocity correction
float Accel_correction[2];       // Complementary Filter acceleration correction
float VelocityXY_base[2];
float VelocityXY_error[2];	   //Velocity error calculation
float Est_AccelXY[2];
//float Old_accelXY[2] = { 0.0, 0.0 };

float Estimated_VelocityXY[2] = { 0, 0 };
float Estimated_PositionXY[2] = { 0, 0 };

// Complementary filter gains and time constant (TC = 2.0)
#define TC_OPTIC 1.2
float k_accl = (float)1/((float)TC_OPTIC*(float)TC_OPTIC);
float k_vel = (float)3/((float)TC_OPTIC);

void PositionEstimate(uint32_t currentTime)
{
	static uint8_t accel_tracker = 0;
	static uint32_t previousTime = 0;
	uint8_t i;
	//int16_t VdesiredXY[2];
	//int32_t VelocityHold_XY[2];


	if ((currentTime - previousTime) < 1*MILLI_SEC)	//1ms
		return;


	previousTime = currentTime;

	// Accelerometer new data check
	if(accel_tracker != imu_tick)
	{
		static uint32_t accTime = 0;
		float Accel_XY[2] = {0,0};
		int16_t

		accel_tracker = imu_tick; 											//Update acclerometer_tick
		float dt = ((float)currentTime - (float)accTime) / 1000000.0f;     	//~5ms sec
		accTime = currentTime;

		//Convert to cm/s2
		Accel_XY[0] = accel_ned.V.X * accVelScale;		//0 -> X Axis
		Accel_XY[1] = accel_ned.V.Y * accVelScale;		//1 -> Y Axis

		//Constraint to 0.81g
		Accel_XY[0] = constrainf(Accel_XY[0], -800, 800);
		Accel_XY[1] = constrainf(Accel_XY[1], -800, 800);

	    //UpdateAccelXY(); // Acceleration update

		if(NewOpticFlowVelocity())
    		updateFromOpticFlow();

	    for (i = 0; i < 2; i++){

			//Correction terms calculation
	    	Accel_correction[i] +=  VelocityXY_error[i] * k_accl * dt;
			VelocityXY_correction[i] +=  VelocityXY_error[i] * k_vel * dt;

			Accel_XY[i] += Accel_correction[i];			//Bias correction for Acc

			//V = U + at
			VelocityXY_base[i] += Accel_XY[i] *dt;

			Estimated_VelocityXY[i] = VelocityXY_base[i] + VelocityXY_correction[i];

			//S = S0 + ut + 1/2at2
			Estimated_PositionXY[i] += Estimated_VelocityXY[i]*dt + 0.5*Accel_XY[i]*dt*dt;

	    }
	    // Debug Parameters
	    debugAccelvar = Accel_XY[0];
	    debugAccelvar1 = Accel_XY[1];
	    debugAccelvar2 = Accel_correction[0];
	    debugAccelvar3 = Accel_correction[1];

	    if(!ARMING_FLAG(ARMED))
	    	resetFilter();

	    addVelocityXY(VelocityXY_base[0], VelocityXY_base[1]);		//Queue up the acc based velocities for calculating error

	    PositionX = lrintf(Estimated_PositionXY[0]);
		PositionY = lrintf(Estimated_PositionXY[1]);

		VelocityX = lrintf(Estimated_VelocityXY[0]);
		VelocityY = lrintf(Estimated_VelocityXY[1]);

		debugOpticFlowVar = (int16_t)PositionX;
		debugOpticFlowVar1 = (int16_t)PositionY;
		//debugOpticFlowVar2 = (int16_t)VelocityX;
		//debugOpticFlowVar3 = (int16_t)VelocityY;
		/*debugOpticFlowVar4 = (int16_t)Accel_correction[0];
		debugOpticFlowVar5 = (int16_t)Accel_correction[1];*/

		// conditions
		//VelHoldAdjustmentXY[i] = VelocityHoldThrottleAdjustment(x_velocity,y_velocity,Est_AccelXY[1],Old_accelXY[1]);
		if(controlSwitch){
			TranslationController(Accel_XY);
		}
		else{
			PID_x = 0;
			PID_y = 0;
			VdesiredX = 0;
			VdesiredY = 0;
		} // Have to reset integral controller
	}//IMU tick loop
}


// Velocity error calculation
void updateFromOpticFlow(void){

	float hist_VelocityXY_base[2];

	if(isVelocityXYBaseQueueFull())
	{
		getFrontVelocityXY(&hist_VelocityXY_base[0], &hist_VelocityXY_base[1]);
		//VelocityXY_error[0] = OpticFlow_Velocity_X - (hist_VelocityXY_base[0] + VelocityXY_correction[0]);
		//VelocityXY_error[1] = OpticFlow_Velocity_Y - (hist_VelocityXY_base[1] + VelocityXY_correction[1]);
		VelocityXY_error[0] = OpticFlow_Vel_X_LPF - (hist_VelocityXY_base[0] + VelocityXY_correction[0]);
		VelocityXY_error[1] = OpticFlow_Vel_Y_LPF - (hist_VelocityXY_base[1] + VelocityXY_correction[1]);
	}

	else
	{
		VelocityXY_error[0] = 0;
		VelocityXY_error[1] = 0;
	}
	OpticFlowVelocity_NewData = false;

}


void resetFilter(void){
	uint8_t i;

	for (i = 0; i < 2; i++){
	//TODO: Figure out which variables to reset

	//Accel_correction[i] = 0;
	//VelocityXY_correction[i] = 0;
	VelocityXY_error[i] = 0;
	Estimated_VelocityXY[i] = 0;
	VelocityXY_base[i] = 0;
	Estimated_PositionXY[i] = 0;
	}

}

#endif






