
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
  
 Mikrokopter Command Lookup Table Implementation
 
 Copyright (C) 2013 Tobias Simon, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include "commands.h"


static const int commands_in[] =
{
   IN_FC_EXTERN_CONTROL,
   IN_FC_DATA_3D,
   IN_FC_DEBUG,
   IN_FC_DISPLAY_REQ_KEY,
   IN_FC_DISPLAY_REQ_MENU,
   IN_FC_MIXER_QUERY,
   IN_FC_MIXER_WRITE,
   IN_FC_PPM_CHANNELS,
   IN_FC_SETTINGS_REQUEST,
   IN_FC_SETTINGS_WRITE,
   IN_FC_ENGINE_TEST,
   IN_FC_VERSION,
   IN_FC_ACK,
   IN_M3_YAW
};


const char *COMMAND_NAMES_IN[] =
{
   "FC_EXTERN_CONTROL",
   "FC_DATA_3D",
   "FC_DEBUG",
   "FC_DISPLAY_REQ_KEY",
   "FC_DISPLAY_REQ_MENU",
   "FC_MIXER_QUERY",
   "FC_MIXER_WRITE",
   "FC_PPM_CHANNELS",
   "FC_SETTINGS_REQUEST",
   "FC_SETTINGS_WRITE",
   "FC_ENGINE_TEST",
   "FC_VERSION",
   "FC_ACK",
   "MK3MAG_YAW"
};


int in_command_exists(int value)
{
   int ret = binsearch(value, commands_in,
                       sizeof(commands_in) / sizeof(int));
   return ret != -1;
}

