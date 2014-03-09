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
  
 MPU6050 Driver Implementation

 Copyright (C) 2014 Tobias Simon, Ilmenau University of Technology
 Copyright (C) 2013 Jan Roemisch, Ilmenau University of Technology

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


#ifndef __MPU6050_H__
#define __MPU6050_H__


#include <util.h>

#include "../../../geometry/quat.h"
#include "../../bus/i2c/i2c.h"


/* Digital low pass filter (dlpf) settings

    Accelerometer            Gyroscope
     (Fs = 1kHz)
   Bandwidth  Delay    Bandwidth  Delay    Fs
        (Hz)    (ms)        (Hz)   (ms)  (kHz)
0        260      0          256   0.98     8 
1        184    2.0          188    1.9     1 
2         94    3.0           98    2.8     1 
3         44    4.9           42    4.8     1 
4         21    8.5           20    8.3     1 
5         10   13.8           10   13.4     1 
6          5   19.0            5   18.6     1 

*/


typedef enum
{
   MPU6050_DLPF_CFG_260_256Hz,
   MPU6050_DLPF_CFG_184_188Hz,
   MPU6050_DLPF_CFG_94_98Hz,
   MPU6050_DLPF_CFG_44_42Hz,
   MPU6050_DLPF_CFG_21_20Hz,
   MPU6050_DLPF_CFG_10_10Hz,
   MPU6050_DLPF_CFG_5_5Hz
}
mpu6050_dlpf_cfg_t;


/* gyro full scale range in deg/sec */
typedef enum
{
   MPU6050_FS_SEL_250,  /* 131 LSB/Â°/s */
   MPU6050_FS_SEL_500,  /* 65.5 LSB/Â°/s */
   MPU6050_FS_SEL_1000, /* 32.8 LSB/Â°/s */
   MPU6050_FS_SEL_2000, /* 16.4 LSB/Â°/s */
}
mpu6050_fs_sel_t;


/* acc full scale range in g */
typedef enum
{
   MPU6050_AFS_SEL_2G,  /* 16384 LSB/g */
   MPU6050_AFS_SEL_4G,  /* 8192 LSB/g */
   MPU6050_AFS_SEL_8G,  /* 4096 LSB/g */
   MPU6050_AFS_SEL_16G, /* 2048 LSB/g */
}
mpu6050_afs_sel_t;


typedef struct
{
   i2c_dev_t i2c_dev;

   mpu6050_fs_sel_t   gfs;
   mpu6050_afs_sel_t  afs;
}
mpu6050_t;


int mpu6050_init(mpu6050_t *mpu, i2c_bus_t *bus, mpu6050_dlpf_cfg_t dlpf, mpu6050_fs_sel_t fs_sel, mpu6050_afs_sel_t afs_sel);

/* read gyro, acc and temperature; parameters may be NULL */
int mpu6050_read(mpu6050_t *mpu, vec3_t *gyro, vec3_t *acc, float *temperature);


#endif /* __MPU6050_H__ */

