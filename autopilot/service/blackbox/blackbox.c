
#include "blackbox.h"

#include <scl.h>
#include <util.h>
#include <msgpack.h>


#define PACKI(val) msgpack_pack_int(pk, val) /* pack integer */
#define PACKF(val) msgpack_pack_float(pk, val) /* pack float */
#define PACKFV(ptr, n) FOR_N(i, n) PACKF(ptr[i]) /* pack float vector */


static void *blackbox_socket = NULL;
static msgpack_sbuffer *msgpack_buf = NULL;
static msgpack_packer *pk = NULL;


static char *blackbox_spec[] = {
   "dt", /* time delta */
   "gyro_x", "gyro_y", "gyro_z", /* gyro */
   "acc_x", "acc_y", "acc_z", /* acc */
   "mag_x", "mag_y", "mag_z", /* mag */
   "lat", "lon", /* gps */
   "ultra", "baro", /* ultra / baro */
   "voltage", /* voltage */
   "rc_pitch", "rc_roll", "rc_yaw", "rc_gas", "rc_sw_l", "rc_sw_r", /* rc */
   "sensor_status" /* sensors */
};


void blackbox_init(void)
{
   ASSERT_ONCE();
   ASSERT_NULL(blackbox_socket);
   /* get scl socket */
   blackbox_socket = scl_get_socket("blackbox");
   ASSERT_NOT_NULL(blackbox_socket);

   /* send blackbox header: */
   ASSERT_NULL(msgpack_buf);
   msgpack_buf = msgpack_sbuffer_new();
   ASSERT_NOT_NULL(msgpack_buf);
   ASSERT_NULL(pk);
   pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   ASSERT_NOT_NULL(pk);
   msgpack_pack_array(pk, ARRAY_SIZE(blackbox_spec));
   
   ASSERT_NOT_NULL(blackbox_spec);
   FOR_EACH(i, blackbox_spec)
   {
      size_t len = strlen(blackbox_spec[i]);
      msgpack_pack_raw(pk, len);
      msgpack_pack_raw_body(pk, blackbox_spec[i], len);
   }
   scl_copy_send_dynamic(blackbox_socket, msgpack_buf->data, msgpack_buf->size);
}

void blackbox_record(float dt,
               marg_data_t *marg_data,
               gps_data_t *gps_data,
               float ultra,
               float baro,
               float voltage,
               float channels[MAX_CHANNELS],
               uint16_t sensor_status)
{
   msgpack_sbuffer_clear(msgpack_buf);
   msgpack_pack_array(pk, ARRAY_SIZE(blackbox_spec));
   PACKF(dt);
   PACKFV(marg_data->gyro.vec, 3);
   PACKFV(marg_data->acc.vec, 3);
   PACKFV(marg_data->mag.vec, 3);
   PACKF(gps_data->lat); PACKF(gps_data->lon);
   PACKF(ultra); PACKF(baro);
   PACKF(voltage);
   PACKFV(channels, 6);
   PACKI(sensor_status);
   scl_copy_send_dynamic(blackbox_socket, msgpack_buf->data, msgpack_buf->size);
}

