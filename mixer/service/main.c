
#include <stdio.h>
#include <opcd_interface.h>
#include <service.h>
#include <scl.h>
#include <msgpack_reader.h>
#include <util.h>
#include <threadsafe_types.h>
#include <pp_prio.h>
#include <float.h>
#include <math.h>

#include "inv_coupling.h"
#include "coupling_matrix_parser.h"


/* thread that reads the thrust: */
tsfloat_t thrust;
MSGPACK_READER_BEGIN(thrust_reader)
   MSGPACK_READER_LOOP_BEGIN(thrust_reader)
      tsfloat_set(&thrust, root.via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END

/* thread that reads the thrust maximum: */
tsfloat_t thrust_max;
MSGPACK_READER_BEGIN(thrust_max_reader)
   MSGPACK_READER_LOOP_BEGIN(thrust_max_reader)
      tsfloat_set(&thrust_max, root.via.dec);
   MSGPACK_READER_LOOP_END
MSGPACK_READER_END


SERVICE_MAIN_BEGIN("mixer", PP_PRIO_1)
{
   tsfloat_init(&thrust, 0.0f); /* 0N force; safe to start with */
   tsfloat_init(&thrust_max, FLT_MAX); /* allow maximum thrust, can be limited later on */

   char *matrix_def;
   tsfloat_t imtx1;
   tsfloat_t imtx2;
   tsfloat_t imtx3;
   tsfloat_t f_c;
   float mixer[FORCES_AND_MOMENTS][MAX_MOTORS];

   /* read configuration: */
   opcd_param_t params[] =
   {
      {"quad_matrix", &matrix_def},
      {"imtx1", &imtx1},
      {"imtx2", &imtx2},
      {"imtx3", &imtx3},
      {"f_c", &f_c},
      OPCD_PARAMS_END
   };
   opcd_params_apply(".", params);
   int n_motors = coupling_matrix_parser_run(matrix_def, mixer);
   coupling_matrix_print(n_motors, mixer);
   FOR_N(i, n_motors)
   {
      mixer[0][i] *= tsfloat_get(&imtx1); /* gas */
      mixer[1][i] *= tsfloat_get(&imtx2); /* pitch */
      mixer[2][i] *= tsfloat_get(&imtx2); /* roll */
      mixer[3][i] *= tsfloat_get(&imtx3); /* yaw */
   }
   inv_coupling_init(n_motors, mixer);
   
   MSGPACK_READER_START(thrust_reader, "thrust", PP_PRIO_1, "sub");
   MSGPACK_READER_START(thrust_max_reader, "thrust_max", PP_PRIO_3, "sub");
   void *torques_socket = scl_get_socket("torques", "sub");
   THROW_IF(torques_socket == NULL, -EIO);
   void *forces_socket = scl_get_socket("forces", "pub");
   THROW_IF(forces_socket == NULL, -EIO);
   
   /* set-up msgpack packer: */
   MSGPACK_PACKER_DECL_INFUNC();
 
   MSGPACK_READER_SIMPLE_LOOP_BEGIN(torques)
   {
      float forces[MAX_MOTORS];
      float thrust_torques[FORCES_AND_MOMENTS] = {0.0, 0.0, 0.0, 0.0};
      if (root.type == MSGPACK_OBJECT_ARRAY)
      {
         thrust_torques[0] = fmin(tsfloat_get(&thrust_max), tsfloat_get(&thrust));
         FOR_N(i, 3)
            thrust_torques[i + 1] = root.via.array.ptr[i].via.dec;
      
         inv_coupling_calc(forces, thrust_torques);

         msgpack_sbuffer_clear(msgpack_buf);
         msgpack_pack_array(pk, n_motors);
         FOR_N(i, n_motors)
            PACKF(forces[i] * tsfloat_get(&f_c));

         scl_copy_send_dynamic(forces_socket, msgpack_buf->data, msgpack_buf->size);
      }
   }
   MSGPACK_READER_SIMPLE_LOOP_END
}
SERVICE_MAIN_END

