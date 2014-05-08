/*___________________________________________________
 |  _____                       _____ _ _       _    |
 | |  __ \                     |  __ (_) |     | |   |
 | | |__) |__ _ __   __ _ _   _| |__) || | ___ | |_  |
 | |  ___/ _ \ '_ \ / _` | | | |  ___/ | |/ _ \| __| |
 | | |  |  __/ | | | (_| | |_| | |   | | | (_) | |_  |
 | |_|   \___|_| |_|\__, |\__,_|_|   |_|_|\___/ \__| |
 |                   __/ |                           |
 |  GNU/Linux based |___/  Multi-Rotor UAV Autopilot |
 |___________________________________________________|
  
 Force [N] and Voltage [V] to relative ESC Command Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <math.h>

#include "force_to_esc.h"


/*
 * ESC:       Mikrokopter BL-CTRL 1.2
 * Motor:     Robby Roxxy 2827-35
 * Propeller: 10x4.5 inch
 */
float force_to_esc_setup1(float force, float volt)
{
   float a = 609.6137f;
   float b = 1.3154f;
   float c = -1.5f;
   return powf((force / a * powf(volt, c)), 1.0f / b);
}


/*
 * ESC:       HobbyKing 20A UBEC SimonK
 * Motor:     Robby Roxxy 2827-35
 * Propeller: 10x4.5 inch
 */
float force_to_esc_setup2(float force, float volt)
{
   if (force < 0)
      return 0;
   const float a = 1.4618e-07;
   const float b = -5.1569e-03;
   const float c = -2.1677e+00;
   const float d = 1.7561e-04;
   const float e = 4.1540e+01;
   float pwm = ((sqrtf((b + d * volt) * (b + d * volt) - 4.0f * a * (c * volt + e - force)) - b - d * volt) / (2.0f * a));
   return (pwm - 10000.0f) / 10000.0f;
}


/*
 * ESC:       Flyduino HEXFET20A SimonK
 * Motor:     Suppo A2212/13
 * Propeller: 10x4.5 inch
 */
float force_to_esc_setup3(float force, float volt)
{
   if (force < 0)
      return 0;
   const float a = 1.2526e-07f;
   const float b = -3.3937e-03f;
   const float c = -1.3746e+00f;
   const float d = 1.3284e-04f;
   const float e = 2.0807e+01f;
   float pwm = ((sqrtf((b + d * volt) * (b + d * volt) - 4.0f * a * (c * volt + e - force)) - b - d * volt) / (2.0f * a));
   return (pwm - 10000.0f) / 10000.0f;
}

