/*
 * commands.c
 *
 *  Created on: 14.06.2010
 *      Author: tobi
 */


#include <stdlib.h>

#include "commands.h"
#include "util.h"


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
