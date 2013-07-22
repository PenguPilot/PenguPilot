

#include <util.h>
#include <stdio.h>

#include "../../service/platform/drotek_marg2.h"


int main(void)
{
   i2c_bus_t i2c_3;
   i2c_bus_open(&i2c_3, "/dev/i2c-3");
   drotek_marg2_t marg;
   drotek_marg2_init(&marg, &i2c_3);
   printf("acc_x acc_y acc_z mag_x mag_y mag_z\n");
   while (1)
   {
      msleep(10);
      marg_data_t marg_data;
      drotek_marg2_read(&marg_data, &marg);
      printf("%f %f %f %f %f %f\n",
             marg_data.acc.x, marg_data.acc.y, marg_data.acc.z,
             marg_data.mag.x, marg_data.mag.y, marg_data.mag.z);
   }
}

