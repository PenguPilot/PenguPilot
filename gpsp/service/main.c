

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


int main(int argc, char *argv[])
{
   if (scl_init("gpsp") != 0)
   {
      syslog(LOG_CRIT, "could not init scl module");
      exit(EXIT_FAILURE);
   }
   
   opcd_params_init("", 0);
   char *plat = NULL;
   opcd_param_get("platform", &plat);

   char pid_file[1024];
   sprintf(pid_file, "%s/.PenguPilot/run/gpsp.pid", getenv("HOME"));
   if (strcmp(plat, "gumstix_quad") == 0)
   {
      daemonize(pid_file, _main_serial, _cleanup, argc, argv);
   }
   else if (strcmp(plat, "pi_quad") == 0)
   {
      daemonize(pid_file, _main_i2c, _cleanup, argc, argv);   
   }
   return 0;
}


