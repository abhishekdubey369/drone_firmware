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

#pragma once



#ifdef __cplusplus
extern "C" {
#endif

//AdityaLakshmi

// Optic flow sensor data check
//bool isOpticflowDataNew (void);
//extern bool OpticFlowDataNewflag;
bool isOpticFlowDataValid (void);


// Optic flow sensor
extern uint8_t motion_reg;						  // motion register
extern uint8_t observation;						  // observation
extern uint8_t surface_quality;					  // Surface Quality
extern uint16_t shutter;	                      // Shutter
extern int16_t del_X;  							  // del X after the value was swapped properly
extern int16_t del_Y;                             // del Y after the value was swapped properly
extern uint8_t imu_tick;						  // For tracking gyro values


bool initOpticFlow();
//void updateOpticFlow(uint32_t cTime);
bool PollOpticFlow(uint32_t cTime); //Test function to test the frame rate


bool initBRIDGEOpticFlow();
void updateUARTOpticFlow(void);
void updateBRIDGEOpticFlow(uint32_t cTime);


extern int16_t del_X_derotated;
extern int16_t del_Y_derotated;

#ifdef __cplusplus
}
#endif



