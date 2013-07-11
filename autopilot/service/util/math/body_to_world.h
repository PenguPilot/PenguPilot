
/*
 * body_to_world.h
 *
 *  Created on: 22.09.2010
 *      Author: tobi
 */


#ifndef __BODY_TO_WORLD_H__
#define __BODY_TO_WORLD_H__


typedef struct
{
   float pitch;
   float roll;
   float yaw;
}
euler_angles_t;


typedef struct
{
   float pitch_dir; /* towards body "front" is positive */
   float roll_dir; /* towards body "right" is positive */
   float yaw_dir; /* away from body "bottom" is positive */
}
body_vector_t;


typedef struct
{
   float x_dir; /* lon, east is positive */
   float y_dir; /* lat, north is positive */
   float z_dir; /* sky is positive */
}
world_vector_t;


struct body_to_world;
typedef struct body_to_world body_to_world_t;


body_to_world_t *body_to_world_create(void);

void body_to_world_transform(body_to_world_t *btw, euler_angles_t *angles, body_vector_t *in, world_vector_t *out);


#endif /* __BODY_TO_WORLD_H__ */
