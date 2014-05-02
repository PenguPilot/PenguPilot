

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <daemon.h>

#include "main_serial.h"
#include "main_i2c.h"

#include <opcd_interface.h>
#include <scl.h>
#include <syslog.h>

void _cleanup(void)
{

}


int _main(int argc, char *argv[])
{
   if (scl_init("gpsp") != 0)
   {
      syslog(LOG_CRIT, "could not init scl module");
      exit(EXIT_FAILURE);
   }
   
   opcd_params_init("", 0);
   char *plat = NULL;
   opcd_param_get("platform", &plat);
   if (strcmp(plat, "gumstix_quad") == 0)
   {
      main_serial();
   }
   else if (strcmp(plat, "pi_quad") == 0)
   {
      main_i2c();   
   }
   return 0;
}


int main(int argc, char *argv[])
{
   char pid_file[1024];
   sprintf(pid_file, "%s/.PenguPilot/run/gpsp.pid", getenv("HOME"));
   daemonize(pid_file, _main, _cleanup, argc, argv);
   return 0;
}


