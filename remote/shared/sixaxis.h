
#ifndef __SIXAXIS_H__
#define __SIXAXIS_H__


int sixaxis_init(void);

int sixaxis_read(float *pitch, float *roll, float *yaw, float *gas, float *sw_l, float *sw_r);


#endif /* __SIXAXIS_H__ */


