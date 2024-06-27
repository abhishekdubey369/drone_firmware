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
#include "drivers/ranging_vl53l0x.h"
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

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "opticflow.h"

#include "../drivers/opticflow_paw3903.h"

#include "mw.h"


void VelocityFromOpticFlow(void);


int16_t debugOpticFlowVar=0;
int16_t debugOpticFlowVar1=0;
int16_t debugOpticFlowVar2=0;
int16_t debugOpticFlowVar3=0;
int16_t debugOpticFlowVar4=0;
int16_t debugOpticFlowVar5=0;
int16_t debugOpticFlowVar6=0;
int16_t debugOpticFlowVar7=0;
int16_t debugOpticFlowVar8=0;
int16_t debugOpticFlowVar9=0;
int16_t debugOpticFlowVar10=0;
int16_t debugOpticFlowVar11=0;

// AdityaLakshmi
bool OpticFlowVelocity_NewData = false;
float OpticFlow_Velocity_X = 0; // optic flow velocity in X direction
float OpticFlow_Velocity_Y = 0; // optic flow velocity in Y direction
float OpticFlow_Vel_X_LPF = 0;  // optic flow velocity in X direction after LPF
float OpticFlow_Vel_Y_LPF = 0; // optic flow velocity in Y direction after LPF
//Poll the sensor and convert to velocity
void UpdateOpticFlow(uint32_t cTime)
{
	bool valid_data;
	valid_data = PollOpticFlow(cTime);

	if (valid_data)
		VelocityFromOpticFlow();

}


//Optic flow velocity conversion

void VelocityFromOpticFlow()
{
	//float beta = 0.253386941;   // Inverse of focal length in cm
	float beta = 0.2;   // Inverse of focal length in cm
	float surface_alt;			// Laser altitude
	float velocity_factor;		// factor for conversion of optic flow
	float gain_lf = 0.8; 	// Low pass filter gain
	static float prev_OpticFlow_Vel_X_LPF = 0;
	static float prev_OpticFlow_Vel_Y_LPF = 0;
	//static int print_timer2 =0;
	surface_alt = ToF_Height; // later replace it with Estimated surface altitude
	Monitor.println("surface alt: ", surface_alt);

	velocity_factor = 1 + (beta * surface_alt);

	OpticFlow_Velocity_X = velocity_factor * del_X_derotated;
	OpticFlow_Velocity_Y = velocity_factor * del_Y_derotated;
	//if(print_timer2==285){
	//	print_timer2 = 0;

	Monitor.println("OpticFlow_Velocity_X: ", OpticFlow_Velocity_X);
	Monitor.println("OpticFlow_Velocity_Y: ", OpticFlow_Velocity_Y);
	Monitor.println(" ");


	OpticFlow_Vel_X_LPF = gain_lf*prev_OpticFlow_Vel_X_LPF + (1-gain_lf)*OpticFlow_Velocity_X;
	OpticFlow_Vel_Y_LPF = gain_lf*prev_OpticFlow_Vel_Y_LPF + (1-gain_lf)*OpticFlow_Velocity_Y;

	prev_OpticFlow_Vel_X_LPF = OpticFlow_Vel_X_LPF;
	prev_OpticFlow_Vel_Y_LPF = OpticFlow_Vel_Y_LPF;

	debugOpticFlowVar8 = OpticFlow_Vel_X_LPF;
	debugOpticFlowVar9 = OpticFlow_Vel_Y_LPF;
	debugOpticFlowVar10 = del_X_derotated;
	debugOpticFlowVar11 = del_Y_derotated;


	OpticFlowVelocity_NewData = true;
}

bool NewOpticFlowVelocity(void)
{
	return OpticFlowVelocity_NewData;
}










