/*
 * chr-6d.c
 *
 *  Created on: 30.06.2010
 *      Author: tobi, jan
 */


#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <netinet/in.h>

#include "util.h"
#include "chr_6dm.h"
#include "chr_6dm_cmds.h"
#include "chr_6dm_util.h"

#include <threadsafe_types.h>
#include <simple_thread.h>
#include "../../../util/logger/logger.h"
#include "../../../util/math/conv.h"
#include "../../../../../shared/serial.h"
#include <opcd_interface.h>
#include <util.h>

#undef CHR6DM_DEBUG


/*
 * chr6dm thread:
 */
#define CHR6DM_THREAD_NAME       "chr6dm-reader"
#define CHR6DM_THREAD_PRIORITY   0
static simple_thread_t chr6dm_thread;
static serialport_t port;
static ahrs_data_t imu_data;
static pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;

static pthread_mutex_t condition_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t self_test_completed  = PTHREAD_COND_INITIALIZER;
static pthread_cond_t acc_bias_reported  = PTHREAD_COND_INITIALIZER;
static pthread_cond_t acc_ref_vector_reported  = PTHREAD_COND_INITIALIZER;
static pthread_cond_t gyro_bias_reported  = PTHREAD_COND_INITIALIZER;
static pthread_cond_t command_complete  = PTHREAD_COND_INITIALIZER;
static pthread_cond_t process_covar_reported  = PTHREAD_COND_INITIALIZER;
static pthread_cond_t mag_ref_vector_reported  = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t new_data_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t new_data_cond = PTHREAD_COND_INITIALIZER;

static char *device_conf_file;
static char *serial_port;
static tsfloat_t process_covar;
static tsfloat_t mag_covar;
static tsfloat_t acc_covar;




#define CHR6DM_COMMAND_COMPLETE          (0xB0)
#define CHR6DM_COMMAND_FAILED            (0xB1)
#define CHR6DM_BAD_CHECKSUM              (0xB2)
#define CHR6DM_BAD_DATA_LENGTH           (0xB3)
#define CHR6DM_UNRECOGNIZED_PACKET       (0xB4)
#define CHR6DM_BUFFER_OVERFLOW           (0xB5)
#define CHR6DM_STATUS_REPORT             (0xB6)
#define CHR6DM_SENSOR_DATA               (0xB7)
#define CHR6DM_GYRO_BIAS_REPORT          (0xB8)
#define CHR6DM_GYRO_SCALE_REPORT         (0xB9)
#define CHR6DM_START_CAL_REPORT          (0xBA)
#define CHR6DM_ACC_BIAS_REPORT           (0xBB)
#define CHR6DM_ACC_REF_VECTOR_REPORT     (0xBC)
#define CHR6DM_ACTIVE_CHANNEL_REPORT     (0xBD)
#define CHR6DM_ACC_COVARIANCE_REPORT     (0xBE)
#define CHR6DM_MAG_COVARIANCE_REPORT     (0xBF)
#define CHR6DM_PROCESS_COVARIANCE_REPORT (0xC0)
#define CHR6DM_STATE_COVARIANCE_REPORT   (0xC1)
#define CHR6DM_EKF_CONFIG_REPORT         (0xC2)
#define CHR6DM_GYRO_ALIGNMENT_REPORT     (0xC3)
#define CHR6DM_ACC_ALIGNMENT_REPORT      (0xC4)
#define CHR6DM_MAG_REF_VECTOR_REPORT     (0xC5)
#define CHR6DM_MAG_CAL_REPORT            (0xC6)
#define CHR6DM_MAG_BIAS_REPORT           (0xC7)
#define CHR6DM_BROADCAST_MODE_REPORT     (0xC8)


int initialized = 0;


void chr6dm_wait_for_data(void)
{
   pthread_mutex_lock(&new_data_mutex);
   pthread_cond_wait(&new_data_cond, &new_data_mutex);
   pthread_mutex_unlock(&new_data_mutex);
}


int chr6dm_read(ahrs_data_t *data_out)
{
   ASSERT_TRUE(chr6dm_thread.running);

   pthread_mutex_lock(&data_mutex);
   *data_out = imu_data;
   pthread_mutex_unlock(&data_mutex);
   return 0;
}


static float float_from_netfloat(float local_float)
{
   float f;
   void *temp = &local_float;
   unsigned int i = ntohl(*((unsigned int *)temp));
   memcpy(&f, &i, sizeof(float));
   return f;
}



int bias_acc_pitch = 0;
int bias_acc_roll = 0;
int bias_acc_yaw = 0;

float acc_pitch_scale = 0;
float acc_roll_scale = 0;
float acc_yaw_scale = 0;



static void chr6dm_handle_packet(unsigned char type, unsigned char *data)
{
   ASSERT_NOT_NULL(data);

   switch (type)
   {
      case CHR6DM_COMMAND_COMPLETE:
         LOG(LL_DEBUG, "command complete: %s", chr6dm_get_rx_command_name(data[0]));
         pthread_mutex_lock(&condition_mutex);
         pthread_cond_signal(&command_complete);
         pthread_mutex_unlock(&condition_mutex);
         break;

      case CHR6DM_COMMAND_FAILED:
         LOG(LL_ERROR, "command failed: %s!", chr6dm_get_rx_command_name(data[0]));
         break;

      case CHR6DM_BAD_CHECKSUM:
         LOG(LL_ERROR, "bad checksum!");
         break;

      case CHR6DM_BAD_DATA_LENGTH:
         LOG(LL_ERROR, "bad data length for command: %s!", chr6dm_get_rx_command_name(data[0]));
         break;

      case CHR6DM_UNRECOGNIZED_PACKET:
         LOG(LL_WARNING, "unrecognized packet type: %X!", data[0]);
         break;

      case CHR6DM_BUFFER_OVERFLOW:
         LOG(LL_ERROR, "buffer overflow!");
         break;

      case CHR6DM_STATUS_REPORT:
      {
         int error = 0;
         for (int bit = 0; bit < 6; bit++)
         {
            if (data[0] & (1 << bit))
            {
               error = 1;
               LOG(LL_ERROR, "self-test failed for: %s_%c",
                   bit > 2 ? "GYRO" : "ACC", (bit % 3) + 'X');
            }
         }
         if (error)
         {
            exit(1);
         }
         LOG(LL_INFO, "self-test passed");
         pthread_mutex_lock(&condition_mutex);
         pthread_cond_signal(&self_test_completed);
         pthread_mutex_lock(&condition_mutex);
         break;
      }

      case CHR6DM_SENSOR_DATA:
      {
         if (initialized)
         {
            unsigned short act_channels = (data[0] << 8) | data[1];
            float sensor_array[CHR6DM_N_CHANNELS];
            int i = 2;
            for (int channel = CHR6DM_YAW; channel >= CHR6DM_ACC_Z; channel--)
            {
               if (act_channels & CHR6DM_CHANNEL_TO_BIT(channel))
               {
                  sensor_array[channel] = ((float)((short)((data[i] << 8) | data[i + 1]))) * chr6dm_scale_table_entry(channel);
                  i += 2;
               }
               else
               {
                  sensor_array[channel] = NAN;
               }
            }

            pthread_mutex_lock(&data_mutex);
            imu_data.pitch = deg2rad(sensor_array[CHR6DM_PITCH]);
            imu_data.roll = deg2rad(sensor_array[CHR6DM_ROLL]);
            imu_data.yaw = deg2rad(sensor_array[CHR6DM_YAW]);
            imu_data.pitch_rate = sensor_array[CHR6DM_PITCH_RATE];
            imu_data.roll_rate = sensor_array[CHR6DM_ROLL_RATE];
            imu_data.yaw_rate = sensor_array[CHR6DM_YAW_RATE];
            

            imu_data.acc_pitch = sensor_array[CHR6DM_ACC_X] * acc_pitch_scale;
            imu_data.acc_roll = sensor_array[CHR6DM_ACC_Y] * acc_roll_scale;
            imu_data.acc_yaw = sensor_array[CHR6DM_ACC_Z] * acc_yaw_scale;
            pthread_mutex_unlock(&data_mutex);

            pthread_mutex_lock(&new_data_mutex);
            pthread_cond_broadcast(&new_data_cond); /* signal new data available */
            pthread_mutex_unlock(&new_data_mutex);
            break;
         }
      }

      case CHR6DM_GYRO_BIAS_REPORT:
         LOG(LL_DEBUG, "CHR6DM_GYRO_BIAS_REPORT");
         pthread_mutex_lock(&condition_mutex);
         pthread_cond_signal(&gyro_bias_reported);
         pthread_mutex_unlock(&condition_mutex);
         break;

      case CHR6DM_GYRO_SCALE_REPORT:
         LOG(LL_DEBUG, "CHR6DM_GYRO_SCALE_REPORT");
         break;

      case CHR6DM_START_CAL_REPORT:
         LOG(LL_DEBUG, "CHR6DM_START_CAL_REPORT");
         break;

      case CHR6DM_ACC_BIAS_REPORT:
      {
         short yaw_bias = (short)((data[0] << 8) | data[1]);
         short roll_bias = (short)((data[2] << 8) | data[3]);
         short pitch_bias = (short)((data[4] << 8) | data[5]);
         LOG(LL_DEBUG, "CHR6DM_ACC_BIAS_REPORT: pitch = %d, roll = %d, yaw = %d", pitch_bias, roll_bias, yaw_bias);

         pthread_mutex_lock(&condition_mutex);
         pthread_cond_signal(&acc_bias_reported);
         pthread_mutex_unlock(&condition_mutex);
         break;
      }

      case CHR6DM_ACC_REF_VECTOR_REPORT:
      {
         short z_acc_ref = (short)((data[0] << 8) | data[1]);
         short y_acc_ref = (short)((data[2] << 8) | data[3]);
         short x_acc_ref = (short)((data[4] << 8) | data[5]);
         LOG(LL_DEBUG, "CHR6DM_ACC_REF_VECTOR_REPORT: x = %d, y = %d, z = %d", x_acc_ref, y_acc_ref, z_acc_ref);

         pthread_mutex_lock(&condition_mutex);
         pthread_cond_signal(&acc_ref_vector_reported);
         pthread_mutex_unlock(&condition_mutex);
         break;
      }

      case CHR6DM_ACTIVE_CHANNEL_REPORT:
         LOG(LL_DEBUG, "CHR6DM_ACTIVE_CHANNEL_REPORT");
         break;

      case CHR6DM_ACC_COVARIANCE_REPORT:
      {
         LOG(LL_DEBUG, "CHR6DM_ACC_COVARIANCE_REPORT: %f", float_from_netfloat(*(float *)data));
         break;
      }

      case CHR6DM_MAG_COVARIANCE_REPORT:
      {
         float covar;
         memcpy(&covar, data, sizeof(float));
         LOG(LL_DEBUG, "CHR6DM_MAG_COVARIANCE_REPORT: %f", float_from_netfloat(*(float *)data));
         break;
      }

      case CHR6DM_PROCESS_COVARIANCE_REPORT:
      {
         float covar;
         memcpy(&covar, data, sizeof(float));
         LOG(LL_DEBUG, "CHR6DM_PROCESS_COVARIANCE_REPORT: %f", float_from_netfloat(*(float *)data));

         pthread_mutex_lock(&condition_mutex);
         pthread_cond_signal(&process_covar_reported);
         pthread_mutex_unlock(&condition_mutex);

         break;
      }

      case CHR6DM_STATE_COVARIANCE_REPORT:
         LOG(LL_DEBUG, "CHR6DM_STATE_COVARIANCE_REPORT");
         break;

      case CHR6DM_EKF_CONFIG_REPORT:
         LOG(LL_DEBUG, "CHR6DM_EKF_CONFIG_REPORT");
         break;

      case CHR6DM_GYRO_ALIGNMENT_REPORT:
         LOG(LL_DEBUG, "CHR6DM_GYRO_ALIGNMENT_REPORT");
         break;

      case CHR6DM_ACC_ALIGNMENT_REPORT:
         LOG(LL_DEBUG, "CHR6DM_ACC_ALIGNMENT_REPORT");
         break;

      case CHR6DM_MAG_REF_VECTOR_REPORT:
      {
         short z_mag_ref = (short)((data[0] << 8) | data[1]);
         short y_mag_ref = (short)((data[2] << 8) | data[3]);
         short x_mag_ref = (short)((data[4] << 8) | data[5]);
         LOG(LL_DEBUG, "CHR6DM_MAG_REF_VECTOR_REPORT: x = %d, y = %d, z = %d", x_mag_ref, y_mag_ref, z_mag_ref);

         pthread_mutex_lock(&condition_mutex);
         pthread_cond_signal(&mag_ref_vector_reported);
         pthread_mutex_unlock(&condition_mutex);
         break;
      }

      case CHR6DM_MAG_CAL_REPORT:
         LOG(LL_DEBUG, "CHR6DM_MAG_CAL_REPORT");
         break;

      case CHR6DM_MAG_BIAS_REPORT:
         LOG(LL_DEBUG, "CHR6DM_MAG_BIAS_REPORT");
         break;

      case CHR6DM_BROADCAST_MODE_REPORT:
         LOG(LL_DEBUG, "CHR6DM_BROADCAST_MODE_REPORT");
         break;

      default:
         assert(!"program malfunction!");
   }

}


SIMPLE_THREAD_BEGIN(chr6dm_thread_func)
{
   enum
   {
      STATE_READ_S,
      STATE_READ_N,
      STATE_READ_P,
      STATE_READ_TYPE,
      STATE_READ_LEN,
      STATE_READ_DATA,
      STATE_READ_CSUM_1,
      STATE_READ_CSUM_2
   } state = STATE_READ_S;

   unsigned char type = 0;
   unsigned short csum = 0;
   unsigned char csum_h = 0;
   int pl_len = 0;
   int pl_count = 0;
   static unsigned char pl_buffer[256]; /* frame payload buffer */

   SIMPLE_THREAD_LOOP_BEGIN
   {
#define SERIAL_BUF_SIZE 1024
      static char serial_buffer[SERIAL_BUF_SIZE]; /* serial data buffer */

      /*
       * read data from serial port into buffer:
       */
      int buf_len = serial_read_buffer(serial_buffer, SERIAL_BUF_SIZE, &port);
      if (buf_len == -1)
      {
         LOG(LL_ERROR, "could not read from serial port");
         continue;
      }

      /*
       * step through the buffer and let the statemachine do its job:
       */
      for (int pos = 0; pos < buf_len; pos++)
      {
         unsigned char c = serial_buffer[pos];
         if ((state == STATE_READ_S) && (c == 's'))
         {
            state = STATE_READ_N;
            csum = c;
         }
         else if ((state == STATE_READ_N) && (c == 'n'))
         {
            state = STATE_READ_P;
            csum += c;
         }
         else if ((state == STATE_READ_P) && (c == 'p'))
         {
            state = STATE_READ_TYPE;
            csum += c;
         }
         else if (   state == STATE_READ_TYPE
         && c >= CHR6DM_COMMAND_COMPLETE
         && c <= CHR6DM_BROADCAST_MODE_REPORT)
         {
            type = (unsigned char) c;
            state = STATE_READ_LEN;
            csum += c;
         }
         else if (state == STATE_READ_LEN)
         {
            pl_len = c;
            pl_count = 0;
            state = STATE_READ_DATA;
            csum += c;
         }
         else if (state == STATE_READ_DATA)
         {
            if (pl_count < pl_len)
            {
               pl_buffer[pl_count++] = c;
               csum += c;
            }
            else
            {
               state = STATE_READ_CSUM_2;
               csum_h = c;
            }
         }
         else if (state == STATE_READ_CSUM_2)
         {
            /*
             * test checksum and handle data:
             */
            if (csum != ((csum_h << 8) | c))
            {
               LOG(LL_DEBUG, "csum: 0x%4X != 0x%4X", csum, (csum_h << 8) | c);
               if (c == 's')
                  state = STATE_READ_N;
               else
                  state = STATE_READ_S;
            }
            else
            {
               chr6dm_handle_packet(type, pl_buffer);
               state = STATE_READ_S;
            }
         }
         else
         {
            //LOG(LL_DEBUG, "INVAL char: %2X in state: %d", c, state);

            if (c == 's')
               state = STATE_READ_N;
            else
               state = STATE_READ_S;

            /*static int err_count = 0;
            err_count++;
            if (err_count > 1000)
            {
               LOG(LL_INFO, "imu garbage detected!");
               serial_close(&port);
               serial_open(&port, serial_port, 115200, 0, 0, 0);
               err_count = 0;
            }*/
         }
      }
   }
   SIMPLE_THREAD_LOOP_END
}
SIMPLE_THREAD_END



#define LOOP_WHILE_COND_NOT_TRUE(command, condition, seconds) \
   while (1) \
   { \
      command; \
      struct timespec ts; \
      (void)clock_gettime(CLOCK_REALTIME, &ts); \
      ts = timespec_add_s(ts, seconds); \
      int result = pthread_cond_timedwait(& condition, &condition_mutex, &ts); \
      if (result != ETIMEDOUT) \
      { \
         break; \
      } \
   }


int chr6dm_init(void)
{
   static opcd_param_t params[] =
   {
      {"serial_port", &serial_port},
      {"device_conf_file", &device_conf_file},
      {"process_covar", &process_covar},
      {"mag_covar", &mag_covar},
      {"acc_covar", &acc_covar},
      OPCD_PARAMS_END
   };
   opcd_params_apply("sensors.chr_6dm.", params);

   char *home = getenv("MOBICOM_SUBPROJECT_PATH");
   ASSERT_NOT_NULL(home);
   char buffer[1024];
   strcpy(buffer, home);
   strcat(buffer, "/components/core/");
   strcat(buffer, device_conf_file);

   FILE *in = fopen(buffer, "r");
   if (in == NULL)
   {
      LOG(LL_INFO, "could not open device configuration file");
      return -1;
   }

   int mag_ref_x, mag_ref_y, mag_ref_z;

   int ret = fscanf(in, "%d\n%d\n%d\n%f\n%f\n%f\n%d\n%d\n%d", &bias_acc_pitch, &bias_acc_roll, &bias_acc_yaw,
                    &acc_pitch_scale, &acc_roll_scale, &acc_yaw_scale,
                    &mag_ref_x, &mag_ref_y, &mag_ref_z);
   fclose(in);

   LOG(LL_INFO, "loaded biases: acc_pitch: %d, acc_roll: %d, acc_yaw: %d",
       bias_acc_pitch, bias_acc_roll, bias_acc_yaw);

   ret = serial_open(&port, serial_port, 115200, 0, 0, 0);
   if (ret != 0)
   {
      LOG(LL_INFO, "could not open serial port");
      return ret;
   }

   LOG(LL_INFO, "initializing chr-6dm");
   simple_thread_start(&chr6dm_thread, chr6dm_thread_func, CHR6DM_THREAD_NAME, CHR6DM_THREAD_PRIORITY, NULL);

   LOG(LL_DEBUG, "enabling silent mode");
   LOOP_WHILE_COND_NOT_TRUE(chr6dm_set_silent_mode(&port), command_complete, 1);
   
   LOG(LL_DEBUG, "activating channels");
   LOOP_WHILE_COND_NOT_TRUE(chr6dm_set_active_channels(&port, CHR6DM_AUTOPILOT_CHANNELS), command_complete, 1)

   LOG(LL_DEBUG, "calibrating rate gyros");
   LOOP_WHILE_COND_NOT_TRUE(chr6dm_zero_gyros(&port), gyro_bias_reported, 10);

   LOG(LL_DEBUG, "setting process covariance to %f", tsfloat_get(&process_covar));
   LOOP_WHILE_COND_NOT_TRUE(chr6dm_set_process_covar(&port, tsfloat_get(&process_covar)), command_complete, 1);

   LOG(LL_DEBUG, "setting mag covariance to %f", tsfloat_get(&mag_covar));
   LOOP_WHILE_COND_NOT_TRUE(chr6dm_set_mag_covar(&port, 1.0e-9), command_complete, 1);
   
   LOG(LL_DEBUG, "setting acc covariance to %f", tsfloat_get(&acc_covar));
   LOOP_WHILE_COND_NOT_TRUE(chr6dm_set_acc_covar(&port, tsfloat_get(&acc_covar)), command_complete, 1);

   LOG(LL_DEBUG, "resetting extended kalman filter");
   LOOP_WHILE_COND_NOT_TRUE(chr6dm_ekf_reset(&port), command_complete, 1);
   
   LOG(LL_DEBUG, "enabling full-speed broadcast mode");
   LOOP_WHILE_COND_NOT_TRUE(chr6dm_set_broadcast_mode(&port, 0xFF), command_complete, 1);
   initialized = 1;
   
   float last_yaw = 10.0;
   while (fabs(last_yaw - imu_data.yaw) > 0.00001) //(count--)  
   {
      //LOG(LL_INFO, "%f", fabs(last_yaw - imu_data.yaw));
      last_yaw = imu_data.yaw;
      msleep(100);
   }
   
   LOG(LL_DEBUG, "setting mag covariance to %f", tsfloat_get(&mag_covar));
   LOOP_WHILE_COND_NOT_TRUE(chr6dm_set_mag_covar(&port, tsfloat_get(&mag_covar)), command_complete, 1);
   
   LOG(LL_INFO, "chr-6dm up and running");
   
   return 0;
}


int chr6dm_finalize(void)
{
   simple_thread_stop(&chr6dm_thread);
   return serial_close(&port);
}
