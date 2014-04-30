

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <daemon.h>

#include "main_serial.h"
#include "main_i2c.h"



void _cleanup(void)
{

}


int main(int argc, char *argv[])
{
   char pid_file[1024];
   sprintf(pid_file, "%s/.PenguPilot/run/gpsp.pid", getenv("HOME"));
   if (1)
   {
      daemonize(pid_file, _main_serial, _cleanup, argc, argv);
   }
   else
   {
      daemonize(pid_file, _main_i2c, _cleanup, argc, argv);   
   }
   return 0;
}


