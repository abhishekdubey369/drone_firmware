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

#include "command/command.h"
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
#include "flight/opticflow.h"
#include "flight/mixer.h"
#include "flight/altitudehold.h"

#include "config/runtime_config.h"

#include "posEstimate.h"

#include "posControl.h"


#include "API/API-Utils.h"
#include "io/rc_controls.h"

#define corr_scale 512/100
#define corr_scale2 1096/100

#define TOLERANCE_XY 10
#define CONTROL_ENDPOINT 100

#define MILLI_SEC 1000

static rcControlsConfig_t *rcControlsConfig;

uint8_t velocityControlX = 1;
uint8_t velocityControlY = 1;
int32_t Poshold;
int32_t error;
int32_t initialPitchHold;
int32_t PitchAdjustment;
int32_t estPosition = 0;
int32_t desiredPositionX = 0;
int32_t desiredPositionY = 0;
int32_t desiredVelo = 0;
int32_t errorAlt;
static uint8_t isPositionXChanged = 1;
static uint8_t isPositionYChanged = 1;
static uint8_t isAltHoldChanged = 0;

uint8_t kp_posx;
uint8_t kp_posy;
uint8_t kp_velx;
uint8_t kp_vely;
uint8_t ki_velx;
uint8_t ki_vely;
uint8_t kd_velx;
uint8_t kd_vely;

uint8_t _new_kpx;
uint8_t _new_kpy;
uint8_t _new_kdx;
uint8_t _new_kdy;
uint8_t _new_kix;
uint8_t _new_kiy;

int32_t AltHoldfromPos = 100;
int16_t PID_x = 0;
int16_t PID_y = 0;
int16_t _new_IoutX = 0;
int16_t _new_IoutY = 0;
int16_t IoutX = 0;
int16_t IoutY = 0;
int16_t VdesiredX = 0;
int16_t VdesiredY = 0;
uint8_t HeightAchieved = 0;
static pidProfile_t *pidProfile; //PS2

bool controlSwitch = false;
bool PosInit = false;

void configurePosHold(pidProfile_t *initialPidProfile, rcControlsConfig_t *rcControlsConfigPointer) //PS2
{
    pidProfile = initialPidProfile;
    rcControlsConfig = rcControlsConfigPointer;
}

#ifdef OPTIC_FLOW
void TranslationController(float *CompensatedAcc)
{
    int16_t positionXError = 0;
    int16_t positionYError = 0;

    kp_posx = kp_posy = pidProfile->P8[PIDPOS];
    kp_velx = kp_vely = pidProfile->P8[PIDPOSR];
    ki_velx = ki_vely = pidProfile->I8[PIDPOSR];
    kd_velx = kd_vely = pidProfile->D8[PIDPOSR];

    int16_t errorVelX, errorVelY, errorPosX, errorPosY;
    int16_t PoutX, PoutY, DoutX, DoutY;
    static uint8_t position_x_changed, position_y_changed;

    //Position controller


    if(!ARMING_FLAG(ARMED))
    	PosInit = false;


    if(!PosInit && ARMING_FLAG(ARMED)){
		if((ABS(rcData[PITCH] - 1500) < rcControlsConfig->alt_hold_deadband)&&(ABS(rcData[ROLL] - 1500) < rcControlsConfig->alt_hold_deadband))
		{
			if(getEstAltitude() > 60){
				desiredPositionX = PositionX;
				desiredPositionY = PositionY;
				PosInit = true;
			}
		}
	}



    // X-axis
    if (ABS(rcData[PITCH] - 1500) > rcControlsConfig->alt_hold_deadband) {

    	VdesiredX = (rcData[PITCH] - 1500) / 2;
    	VdesiredX = constrain(VdesiredX, -100, 100);
    	position_x_changed = 1;

    }else{
		if(position_x_changed){
			desiredPositionX = PositionX;
			position_x_changed = 0;
		}
		//Position controller
		VdesiredX = kp_posx*(desiredPositionX-PositionX)/10;
		VdesiredX = constrain(VdesiredX, -100, 100);
		if(!PosInit)
		{
			VdesiredX = 0;
		} VdesiredX = 0;

    }


    // Y-Axis
    if (ABS(rcData[ROLL] - 1500) > rcControlsConfig->alt_hold_deadband)//velocity mode
    {
    	VdesiredY = (rcData[ROLL] - 1500) / 2;
    	VdesiredY = -constrain(VdesiredY, -100, 100);
    	position_y_changed = 1;
    }else{

    	if (position_y_changed){
    		desiredPositionY = PositionY;
    		position_y_changed = 0;
    	}
    	VdesiredY = kp_posy*(desiredPositionY-PositionY)/10;
    	VdesiredY = constrain(VdesiredY, -100, 100);
    	if(!PosInit)
    	{
    		VdesiredY = 0;
    	}VdesiredY = 0;
    }

    //VelocityController(setVelocityX, setVelocityY);

    // Velocity Controller
    errorVelX = VdesiredX - VelocityX;
    errorVelY = VdesiredY - VelocityY;

	PoutX = (kp_velx * errorVelX) / 100;
	PoutY = (kp_vely * errorVelY) / 100;

	PoutX = constrain(PoutX, -200, 200);
	PoutY = constrain(PoutY, -200, 200);

	if (ki_velx && ARMING_FLAG(ARMED)) {
		IoutX += (ki_velx * errorVelX) / 100;
		IoutY += (ki_vely * errorVelY) / 100;
		IoutX = constrain(IoutX, -20000, 20000);
		IoutY = constrain(IoutY, -20000, 20000);

	} else { //Take out effects of integral if ki is zero
		IoutX = 0;
		IoutY = 0;
	}


	DoutX = kd_velx *  CompensatedAcc[0];
	DoutY = kd_vely * CompensatedAcc[1];

	PID_x = constrain(PoutX + IoutX / 500 - DoutX, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);
	PID_y = -constrain(PoutY + IoutY / 500 - DoutY, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);

	//debugOpticFlowVar = VdesiredX;
	//debugOpticFlowVar1 = VdesiredY;
	//debugOpticFlowVar2 = DoutX;
	//debugOpticFlowVar3 = DoutY;

	debugOpticFlowVar2 = VdesiredX;
	debugOpticFlowVar3 = VdesiredY;
	debugOpticFlowVar6 = desiredPositionX;
	debugOpticFlowVar7 = desiredPositionY;

}
#endif

//void VelocityController(int16_t VdesiredX, int16_t VdesiredY)
//{





    /*switch (localisationType) {

    case LOC_UWB:

        PID_x = constrain(PoutX + IoutX / 500 - DoutX, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);
        PID_y = -constrain(PoutY + IoutY / 500 - DoutY, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);

        break;

    case LOC_WHYCON:

        PID_x = constrain(PoutX + IoutX / 500 - DoutX, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);
        PID_y = -constrain(PoutY + IoutY / 500 - DoutY, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);

        break;

    case LOC_VICON:

        break;

    default:

        PID_x = constrain(PoutX + IoutX / 500 - DoutX, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);
        PID_y = -constrain(PoutY + IoutY / 500 - DoutY, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);

        break;

    }
    */


//}

void resetPosController(void)
{
    isPositionYChanged = 1;
    isPositionXChanged = 1;

}
/*
void PosVelController(int16_t desiredX, int16_t desiredY, int16_t desiredVel)
{

    kp_posx = kp_posy = pidProfile->P8[PIDPOS];

    int16_t errorX, errorY;

    errorX = desiredX - PositionX;
    errorY = desiredY - PositionY;

    VdesiredX = constrain((kp_posx * errorX) / 100, -desiredVel, desiredVel);
    VdesiredY = constrain((kp_posy * errorY) / 100, -desiredVel, desiredVel);

    VelocityController(VdesiredX, VdesiredY);

}
*/

/*
bool PositionController(int16_t desiredX, int16_t desiredY, int16_t desiredZ)
{
    bool Status = false;
    //setdesiredHeight(desiredZ);

    PosVelController(desiredX, desiredY, 50);
    //SimpleController(desiredX, desiredY, 50);
    if ((abs(desiredX - PositionX) < TOLERANCE_XY) && (abs(desiredY - PositionY) < TOLERANCE_XY) && (HeightAchieved == 1)) {
        if (isLocalisationOn) {	//&&ARMING_FLAG(ARMED))
            Status = true;
        } else {
            Status = false;
        }
    }

    return Status;
}
*/

void setdesiredHeight(int32_t desiredZ)
{
    AltHoldfromPos = desiredZ;
}

int32_t getdesiredHeight(void)
{
    return AltHoldfromPos;
}

int16_t getrcDataRoll(void)
{
  return PID_y;
}

int16_t getrcDataPitch(void)
{
  return PID_x;
}

int16_t getDesiredVelocityX()
{
  return VdesiredX;
}

int16_t getDesiredVelocityY()
{

  return VdesiredY;
}

void resetPosIntegral(void)
{	//To Eliminate windup
    _new_IoutX = 0;
    _new_IoutY = 0;
    IoutX = 0;
    IoutY = 0;

}
/*
bool SimpleController(int16_t desiredX, int16_t desiredY, int16_t desiredZ)
{
    int16_t errorX, errorY;
    bool Status = false;

    //setdesiredHeight(desiredZ); //Set Desired Height

    _new_kpx = _new_kpy = pidProfile->P8[PIDPOSR];
    _new_kix = _new_kiy = pidProfile->I8[PIDPOSR];
    _new_kdx = _new_kdy = pidProfile->D8[PIDPOSR];

    errorX = constrain(desiredX - PositionX, -50, 50);
    errorY = constrain(desiredY - PositionY, -50, 50);
    //errorX = desiredX-PositionX;
    //errorY = desiredY-PositionY;

    if (_new_kix && ARMING_FLAG(ARMED) && isLocalisationOn) {
        _new_IoutX += (_new_kix * (errorX)) / 10;
        _new_IoutX = constrain(_new_IoutX, -15000, 15000);

        _new_IoutY += (_new_kiy * (errorY)) / 10;
        _new_IoutY = constrain(_new_IoutY, -15000, 15000);

    } else {
        _new_IoutX = 0;
        _new_IoutY = 0;
    }

    PID_x = (_new_kpx * errorX) / 10 - constrain(_new_kdx * VelocityX, -120, 120) + _new_IoutX / 500;
    PID_y = -((_new_kpy * errorY) / 10 - constrain(_new_kdy * VelocityY, -120, 120) + _new_IoutY / 500);
    PID_x = constrain(PID_x, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);
    PID_y = constrain(PID_y, -CONTROL_ENDPOINT, CONTROL_ENDPOINT);

    //Has the command been executed??
    if ((abs(errorX) < TOLERANCE_XY) && (abs(errorY) < TOLERANCE_XY) && (HeightAchieved == 1)) {
        if (isLocalisationOn) {	//&&ARMING_FLAG(ARMED))
            Status = true;
        } else {
            Status = false;
        }
    }

    return Status;
}
*/
// AdityaLakshmi

#ifdef OPTIC_FLOW

/*
int32_t setVelocityXY;
int32_t VelocityHold;
int32_t ErrorVelocityI;


// Called in posEstimate
int32_t VelocityHoldAdjustment(int32_t Velocity_xy,int32_t VdesiredXY ,float acc_tmp, float acc_old)
{
	int32_t result_XY = 0;
	int32_t error;

	if (!ARMING_FLAG(ARMED))
	{
	        VelocityHold = Velocity_xy;

	}
	// control check ??
	setVelocityXY =  VdesiredXY;

	error = setVelocityXY - Velocity_xy;

	result_XY = constrain((pidProfile->P8[PIDVEL] * error / 32), -300, +300);


	if (ARMING_FLAG(ARMED))
	{
	        ErrorVelocityI += (pidProfile->I8[PIDVEL] * error);

	    } else
	    {
	        ErrorVelocityI = 0;


	    }
	// change limits
	ErrorVelocityI = constrain(ErrorVelocityI, -(8192 * 300), (8192 * 300));


	result_XY += ErrorVelocityI / 8192;


	result_XY -= constrain(pidProfile->D8[PIDVEL] * (acc_tmp + acc_old) / 512, -150, 150);

 return result_XY;
}
*/
#endif
