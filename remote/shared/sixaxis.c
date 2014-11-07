
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <linux/joystick.h>


#include "sixaxis.h"


#define NAME_LENGTH 128


int fd;
int *axis;
int *button;


int sixaxis_init(void)
{
   unsigned char axes = 2;
   unsigned char buttons = 2;
   int version = 0x000800;
   char name[NAME_LENGTH] = "Unknown";

   if ((fd = open("/dev/input/js0", O_RDONLY)) < 0)
      return fd;

   ioctl(fd, JSIOCGVERSION, &version);
   ioctl(fd, JSIOCGAXES, &axes);
   ioctl(fd, JSIOCGBUTTONS, &buttons);
   ioctl(fd, JSIOCGNAME(NAME_LENGTH), name);
	fcntl(fd, F_SETFL, O_NONBLOCK);

   axis = calloc(axes, sizeof(int));
   button = calloc(buttons, sizeof(char));


   return 0;
}




int sixaxis_read(float *pitch, float *roll, float *yaw, float *gas, float *sw_l, float *sw_r)
{
   struct js_event js;

   if (read(fd, &js, sizeof(struct js_event)) != sizeof(struct js_event))
      return -EIO;

   switch(js.type & ~JS_EVENT_INIT)
   {
      case JS_EVENT_BUTTON:
         button[js.number] = js.value;
         break;

      case JS_EVENT_AXIS:
         axis[js.number] = js.value;
         break;
   }

   *yaw = (float)axis[0] / 32767.0f;
   *gas = (float)axis[1] / 32767.0f;
   *roll = (float)axis[2] / 32767.0f;
   *pitch = (float)axis[3] / 32767.0f;
   if (button[8])
      *sw_l = -1.0;
   if (button[10])
      *sw_l = 1.0;

   if (button[12])
      *sw_r = -1.0f;
   if (button[13])
      *sw_r = 0.0f;
   if (button[14])
      *sw_r = 1.0f;

   return 0;
}
