/*
 * data_formats.h
 *
 *  Created on: 19.06.2010
 *      Author: tobi
 */

#ifndef DATA_FORMATS_H_
#define DATA_FORMATS_H_


struct ExternControl
{
   unsigned char digital[2];
   unsigned char remote_keys;
   signed char pitch;
   signed char roll;
   signed char yaw;
   unsigned char gas;
   signed char height;
   unsigned char free;
   unsigned char frame;
   unsigned char config;
} __attribute__((packed));


typedef struct
{
   unsigned char digital[2];
   short int analog[32];
}
fc_debug_t;


#endif /* DATA_FORMATS_H_ */
