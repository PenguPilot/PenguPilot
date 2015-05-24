
#include <stdio.h>
#include <opcd_interface.h>
#include <service.h>
#include <scl.h>
#include <msgpack_reader.h>
#include <util.h>

#include "inv_coupling.h"
#include "coupling_matrix_parser.h"


SERVICE_MAIN_BEGIN("mixer", 99)
{
   char *matrix_def;
   float mixer[FORCES_AND_MOMENTS][MAX_MOTORS];

   opcd_params_init("mixer", 0);
   /* read configuration: */
   opcd_param_t params[] =
   {
      {"quad_matrix", &matrix_def},
      OPCD_PARAMS_END
   };
   opcd_params_apply(".", params);
   int n_motors = coupling_matrix_parser_run(matrix_def, mixer);
   coupling_matrix_print(n_motors, mixer);
   inv_coupling_init(n_motors, mixer);

   void *moments_socket = scl_get_socket("moments", "sub");
   THROW_IF(moments_socket == NULL, -EIO);
   void *forces_socket = scl_get_socket("forces", "pub");
   THROW_IF(forces_socket == NULL, -EIO);
   
   /* init msgpack buffers: */
   msgpack_sbuffer *msgpack_buf = msgpack_sbuffer_new();
   THROW_IF(msgpack_buf == NULL, -ENOMEM);
   msgpack_packer *pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
   THROW_IF(pk == NULL, -ENOMEM);
   float *forces = malloc(sizeof(float) * n_motors);

   MSGPACK_READER_SIMPLE_LOOP_BEGIN(moments)
   {
      float moments[4] = {0.0, 0.0, 0.0, 0.0};
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         FOR_N(i, 3)
            moments[i] = root.via.array.ptr[i].via.dec;
      
         inv_coupling_calc(forces, moments);

         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, n_motors);
         PACKFV(forces, n_motors);

         scl_copy_send_dynamic(forces_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
   MSGPACK_READER_SIMPLE_LOOP_END
}
SERVICE_MAIN_END

