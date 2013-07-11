/*
 * commands.h
 *
 *  Created on: 14.06.2010
 *      Author: tobi
 */


#ifndef COCO_COMMANDS_H
#define COCO_COMMANDS_H


#include "addresses.h"


#define MERGE_ADDR_CMD(a, c) \
   ((a) << 8 | (c))

#define GET_ADDR(word) \
   ((word) >> 8)

#define GET_CMD_CHAR(word) \
   ((word) & 0xFF)


typedef enum
{
   OUT_FC_ANALOG_LABEL_VALUES = MERGE_ADDR_CMD(ADDR_FC, 'a'),
   OUT_FC_EXTERN_CONTROL      = MERGE_ADDR_CMD(ADDR_FC, 'b'),
   OUT_FC_DISPLAY_REQ_KEY     = MERGE_ADDR_CMD(ADDR_FC, 'h'),
   OUT_FC_DISPLAY_REQ_MENU    = MERGE_ADDR_CMD(ADDR_FC, 'l'),
   OUT_FC_VERSION             = MERGE_ADDR_CMD(ADDR_FC, 'v'),
   OUT_FC_DEBUG               = MERGE_ADDR_CMD(ADDR_FC, 'd'),
   OUT_FC_RESET               = MERGE_ADDR_CMD(ADDR_FC, 'R'),
   OUT_FC_COMPASS_YAW     = MERGE_ADDR_CMD(ADDR_FC, 'K'),
   OUT_FC_ENGINE_TEST         = MERGE_ADDR_CMD(ADDR_FC, 't'),
   OUT_FC_PPM_CHANNELS        = MERGE_ADDR_CMD(ADDR_FC, 'p'),
   OUT_FC_DATA_3D             = MERGE_ADDR_CMD(ADDR_FC, 'c'),
   OUT_FC_MIXER_QUERY         = MERGE_ADDR_CMD(ADDR_FC, 'n'),
   OUT_FC_MIXER_WRITE         = MERGE_ADDR_CMD(ADDR_FC, 'm'),
   OUT_FC_CHANGE_SETTING      = MERGE_ADDR_CMD(ADDR_FC, 'f'),
   OUT_FC_SERIAL_POTI         = MERGE_ADDR_CMD(ADDR_FC, 'y'),
   OUT_FC_SETTINGS_REQUEST    = MERGE_ADDR_CMD(ADDR_FC, 'q'),
   OUT_FC_SETTINGS_WRITE      = MERGE_ADDR_CMD(ADDR_FC, 's'),
   OUT_FC_MOTORS_ACTION       = MERGE_ADDR_CMD(ADDR_FC, 'x'),
   OUT_M3_YAW             = MERGE_ADDR_CMD(ADDR_M3, 'w'),
   OUT_NC_REDIRECT_UART       = MERGE_ADDR_CMD(ADDR_NC, 'u')
}
out_cmd_t;


typedef enum
{
   IN_FC_EXTERN_CONTROL   = MERGE_ADDR_CMD(ADDR_FC, 'B'),
   IN_FC_DATA_3D          = MERGE_ADDR_CMD(ADDR_FC, 'C'),
   IN_FC_DEBUG            = MERGE_ADDR_CMD(ADDR_FC, 'D'),
   IN_FC_DISPLAY_REQ_KEY  = MERGE_ADDR_CMD(ADDR_FC, 'H'),
   IN_FC_DISPLAY_REQ_MENU = MERGE_ADDR_CMD(ADDR_FC, 'L'),
   IN_FC_MIXER_QUERY      = MERGE_ADDR_CMD(ADDR_FC, 'M'),
   IN_FC_MIXER_WRITE      = MERGE_ADDR_CMD(ADDR_FC, 'N'),
   IN_FC_PPM_CHANNELS     = MERGE_ADDR_CMD(ADDR_FC, 'P'),
   IN_FC_SETTINGS_REQUEST = MERGE_ADDR_CMD(ADDR_FC, 'Q'),
   IN_FC_SETTINGS_WRITE   = MERGE_ADDR_CMD(ADDR_FC, 'S'),
   IN_FC_ENGINE_TEST      = MERGE_ADDR_CMD(ADDR_FC, 'T'),
   IN_FC_VERSION          = MERGE_ADDR_CMD(ADDR_FC, 'V'),
   IN_FC_ACK              = MERGE_ADDR_CMD(ADDR_FC, 'X'),
   IN_M3_YAW          = MERGE_ADDR_CMD(ADDR_M3, 'w')
}
in_cmd_t;


extern const char *COMMAND_NAMES_IN[];


int in_command_exists(int value);


#endif /* COCO_COMMANDS_H */
