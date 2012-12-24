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
  
 File Purpose

 Copyright (C) 2012 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */



#include "../util/math/linfunc.h"
#include "../control/position/z_ctrl.h"


/* landing state: */
enum
{
   NORMAL,    /* normal mode before landing starts */
   ALT_DEC,   /* setpoint is decremented until minimul hovering altitude */
   SPIN_DOWN, /* altitude control is disabled, gas value is decreased using timer  */
   OFFLINE    /* motors are offline */
}
state = NORMAL;


static float speed = 0.2f;
static float motors_off_alt = 0.2f;
static float motors_off_max = 1.0f;
static float motors_off_timer = 0.0f;
static linfunc_t spin_down_func;


void landing_init(void)
{
   state = NORMAL;
}


int landing_started(void)
{
   return state != NORMAL;
}


int landing_run(float *z_ctrl_gas, float ultra_alt, float baro_alt, float dt)
{
   int motors_enabled;
   switch (state)
   {
      case NORMAL:
      {
         /* normal mode before landing starts */
         z_setpoint_t sp;
         if (ultra_alt < 5.0)
         {
            sp.val = ultra_alt;
            sp.mode_is_ground = 1;
         }
         else
         {
            sp.val = baro_alt;
            sp.mode_is_ground = 0;
         }
         z_ctrl_set_setpoint(sp);
           
         motors_enabled = 1;
         state = ALT_DEC;
         break;
      }

      case ALT_DEC:   
      {
         /* setpoint is decremented until minimul hovering altitude */
         z_setpoint_t sp;
         sp.mode_is_ground = z_ctrl_mode_is_ground();
         sp.val = z_ctrl_get_setpoint() - speed * dt; /* s = s0 - v * dt */
         z_ctrl_set_setpoint(sp);

         if (ultra_alt <= motors_off_alt)
         {
            /* compute spin down function slope: */
            motors_off_timer = motors_off_max;
            vec2_t p1 = {*z_ctrl_gas, 0.0f};
            vec2_t p2 = {0.0f, motors_off_max};
            linfunc_init_points(&spin_down_func, &p1, &p2);
            /* update state: */
            state = SPIN_DOWN;
         }
         motors_enabled = 1;
         break;
      }

      case SPIN_DOWN:
      {
         /* altitude control is disabled, gas value is decreased using timer  */
         motors_off_timer -= dt;
         if (motors_off_timer < 0.0f)
         {
            *z_ctrl_gas = linfunc_calc(&spin_down_func, motors_off_timer);
            state = OFFLINE;
         }
         motors_enabled = 1;
         break;
      }
      
      case OFFLINE:
      {
         /* motors are offline */
         motors_enabled = 0;
         break;
      }
   }
   return motors_enabled;
}


