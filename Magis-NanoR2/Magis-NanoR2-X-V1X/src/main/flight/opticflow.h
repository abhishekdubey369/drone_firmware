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



#ifdef __cplusplus
extern "C" {
#endif


extern int16_t debugOpticFlowVar;
extern int16_t debugOpticFlowVar1;
extern int16_t debugOpticFlowVar2;
extern int16_t debugOpticFlowVar3;
extern int16_t debugOpticFlowVar4;
extern int16_t debugOpticFlowVar5;
extern int16_t debugOpticFlowVar6;
extern int16_t debugOpticFlowVar7;
extern int16_t debugOpticFlowVar8;
extern int16_t debugOpticFlowVar9;
extern int16_t debugOpticFlowVar10;
extern int16_t debugOpticFlowVar11;

//AdityaLakshmi
extern float OpticFlow_Velocity_X;  // Velocity from optic flow sensor after velocity conversion
extern float OpticFlow_Velocity_Y;
extern float OpticFlow_Vel_X_LPF; // Velocity from optic flow sensor after velocity conversion and filteration
extern float OpticFlow_Vel_Y_LPF;

//extern bool OpticFlowDataNewflag;
extern bool OpticFlowVelocity_NewData;

void UpdateOpticFlow(uint32_t cTime);
void VelocityFromOpticFlow();

bool NewOpticFlowVelocity();


#ifdef __cplusplus
}
#endif


