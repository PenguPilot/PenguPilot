
#include "adns_3080.h"

#include <msgpack.h>

#include <daemon.h>
#include <util.h>
#include <scl.h>
#include <opcd_interface.h>
#include <serial.h>

#include "../shared/remote.h"
#include "../shared/sixaxis.h"
#include "../shared/rc_dsl.h"
#include "../shared/sbus_parser.h"


static int running = 1;
static msgpack_sbuffer *msgpack_buf = NULL;
static msgpack_packer *pk = NULL;
static char *platform = NULL;
static void *rc_socket = NULL;


int _main(void)
{
   /* initialize msgpack buffers: */
   msgpack_buf = msgpack_sbuffer_new();
   pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);

   /* initialize SCL: */
   scl_init("ofs");
   rc_socket = scl_get_socket("oflow");

   /* init opcd and get plaform string: */
   //opcd_params_init("", 0);

   int fd = adns3080_init("/dev/spidev0.0");
   while (running)
   {
      uint8_t buf[1500];
      adns3080_read_image(fd, buf);
      msgpack_sbuffer_clear(msgpack_buf);
      msgpack_pack_array(pk, 1);
      PACKI(buf[10]);  /* index 0: valid */
      scl_copy_send_dynamic(rc_socket, msgpack_buf->data, msgpack_buf->size);
   }
   return 0;
}


void _cleanup(void)
{
   running = 0;
}


void main_wrap(int argc, char *argv[])
{
   (void)argc;
   (void)argv;

   exit(-_main());
}


int main(int argc, char *argv[])
{
   //_main();
   char pid_file[1024];
   sprintf(pid_file, "%s/.PenguPilot/run/ofs.pid", getenv("HOME"));
   daemonize(pid_file, main_wrap, _cleanup, argc, argv);
   return 0;
}
