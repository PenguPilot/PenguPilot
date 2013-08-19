
#include <util.h>
#include <sclhelper.h>
#include <pilot.pb-c.h>
#include <periodic_thread.h>

#include "mon.h"


static pthread_mutex_t mon_data_mutex = PTHREAD_MUTEX_INITIALIZER;
static void *mon_socket = NULL;
static MonData mon_data = MON_DATA__INIT;
static periodic_thread_t emitter_thread;


PERIODIC_THREAD_BEGIN(mon_emitter)
{
   PERIODIC_THREAD_LOOP_BEGIN
   {
      pthread_mutex_lock(&mon_data_mutex);
      SCL_PACK_AND_SEND_DYNAMIC(mon_socket, mon_data, mon_data);
      pthread_mutex_unlock(&mon_data_mutex);
   }
   PERIODIC_THREAD_LOOP_END
}
PERIODIC_THREAD_END


void mon_init(void)
{
   /* open monitoring socket: */
   mon_socket = scl_get_socket("mon");
   ASSERT_NOT_NULL(mon_socket);
   int64_t hwm = 1;
   zmq_setsockopt(mon_socket, ZMQ_HWM, &hwm, sizeof(hwm));

   /* create monitoring connection: */
   const struct timespec period = {0, 100 * NSEC_PER_MSEC};
   periodic_thread_start(&emitter_thread, mon_emitter, "mon_thread", 0, period, NULL);
}


void mon_data_set(float x_err, float y_err, float z_err, float yaw_err)
{
   if (pthread_mutex_trylock(&mon_data_mutex) == 0)
   {
      mon_data.x_err = x_err;
      mon_data.y_err = y_err;
      mon_data.z_err = z_err;
      mon_data.yaw_err = yaw_err;
      pthread_mutex_unlock(&mon_data_mutex);
   }
}

