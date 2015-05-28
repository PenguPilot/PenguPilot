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
  
 MAG ADC Calibration Service Implementation

 Copyright (C) 2015 Tobias Simon, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#include <msgpack.h>
#include <scl.h>
#include <service.h>
#include <msgpack_reader.h>

#include "mag_adc_cal.h"



SERVICE_MAIN_BEGIN("mag_adc_cal", 99)
{
   /* set-up msgpack packer: */
   MSGPACK_PACKER_DECL_INFUNC();
  
   /* initialize SCL: */
   void *mag_raw_socket = scl_get_socket("mag_raw", "sub");
   THROW_IF(mag_raw_socket == NULL, -EIO);
   void *mag_adc_cal_socket = scl_get_socket("mag_adc_cal", "pub");
   THROW_IF(mag_adc_cal_socket == NULL, -EIO);

   /* init calibration data: */
   mag_adc_cal_init();
   
   MSGPACK_READER_SIMPLE_LOOP_BEGIN(mag_raw)
   {
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         vec3_t mag;
         vec3_init(&mag);
         FOR_N(i, 3)
            mag.ve[i] = root.via.array.ptr[i].via.dec;
         mag_adc_cal_apply(&mag);
         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, 3);
         PACKFV(mag.ve, 3);
         scl_copy_send_dynamic(mag_adc_cal_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
   MSGPACK_READER_SIMPLE_LOOP_END
}
SERVICE_MAIN_END

