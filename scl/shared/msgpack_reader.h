

#include <util.h>
#include <scl.h>
#include <msgpack.h>
#include <simple_thread.h>


#define MSGPACK_READER_BEGIN(name) \
   static simple_thread_t name##_thread; \
   static void *name##_socket; \
   SIMPLE_THREAD_BEGIN(name)


#define MSGPACK_READER_LOOP_BEGIN(name) \
   SIMPLE_THREAD_LOOP_BEGIN \
   char __buffer[1024]; \
   int __ret = scl_recv_static(name##_socket, __buffer, sizeof(__buffer)); \
   if (__ret > 0) \
   { \
      msgpack_unpacked __msg; \
      msgpack_unpacked_init(&__msg); \
      if (msgpack_unpack_next(&__msg, __buffer, __ret, NULL)) \
      { \
         msgpack_object root = __msg.data;


#define MSGPACK_READER_LOOP_END \
      } \
      msgpack_unpacked_destroy(&__msg); \
   } \
   else \
   { \
      msleep(10); \
   } \
   SIMPLE_THREAD_LOOP_END


#define MSGPACK_READER_SIMPLE_LOOP_BEGIN(name) \
   while(running) \
   { \
      char __buffer[1024]; \
      int __ret = scl_recv_static(name##_socket, __buffer, sizeof(__buffer)); \
      if (__ret > 0) \
      { \
         msgpack_unpacked __msg; \
         msgpack_unpacked_init(&__msg); \
         if (msgpack_unpack_next(&__msg, __buffer, __ret, NULL)) \
         { \
            msgpack_object root = __msg.data;


#define MSGPACK_READER_SIMPLE_LOOP_END \
         } \
         msgpack_unpacked_destroy(&__msg); \
      } \
      else \
      { \
         msleep(10); \
      } \
   }


#define MSGPACK_READER_END \
   SIMPLE_THREAD_END


/* starts the previously declared reader with given scl socket and priority */
#define MSGPACK_READER_START(name, socket_name, prio, type) \
   name##_socket = scl_get_socket(socket_name, type); \
   THROW_IF(name##_socket == NULL, -EIO); \
   simple_thread_start(&name##_thread, name, "name##_reader", prio, NULL);


#define MSGPACK_PROXY_DECL(name) \
   static void *name##_out_socket; \
   MSGPACK_READER_BEGIN(name) \
      MSGPACK_PACKER_DECL_INFUNC(); \
      MSGPACK_READER_LOOP_BEGIN(name) \
         scl_copy_send_dynamic(name##_out_socket,  __buffer, __ret); \
      MSGPACK_READER_LOOP_END \
   MSGPACK_READER_END

#define MSGPACK_PROXY_START(name, in_socket_name, in_type, out_socket_name, out_type, prio) \
   name##_socket = scl_get_socket(in_socket_name, in_type); \
   THROW_IF(name##_socket == NULL, -EIO); \
   name##_out_socket = scl_get_socket(out_socket_name, out_type); \
   THROW_IF(name##_out_socket == NULL, -EIO); \
   simple_thread_start(&name##_thread, name, "name##_proxy", prio, NULL);


