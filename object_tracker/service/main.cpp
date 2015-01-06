#include <cv.h>
#include <highgui.h>

using namespace cv;
extern "C" {
#include <msgpack.h>
#include <daemon.h>
#include <util.h>
#include <scl.h>
#include <pthread.h>
#include <time.h>
}


void MyFilledCircle( Mat img, Point center )
{
 int thickness = 10;
 int lineType = 8;

 circle( img,
         center,
         100/32.0,
         Scalar( 0, 0, 255 ),
         thickness,
         lineType );
}

IplImage* GetThresholdedImage(IplImage* img)
{
     IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
     cvCvtColor(img, imgHSV, CV_BGR2HSV);
     IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);
     cvInRangeS(imgHSV, cvScalar(120, 80, 80), cvScalar(140, 255, 255), imgThreshed);
     cvReleaseImage(&imgHSV);
     return imgThreshed;
}
 

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
float yaw;
float pitch;
float roll;


void *thread(void *args)
{
   void *ap_socket = scl_get_socket("ap_mon");
   while (1)
   {
      char buffer[1024];
      int ret = scl_recv_static(ap_socket, buffer, sizeof(buffer));
      if (ret > 0)
      {
         msgpack_unpacked msg;
         msgpack_unpacked_init(&msg);
         if (msgpack_unpack_next(&msg, buffer, ret, NULL))
         {
            float array[11];
            /* read received raw channels message: */
            msgpack_object root = msg.data;
            assert (root.type == MSGPACK_OBJECT_ARRAY);
            int n_channels = root.via.array.size - 1;
            int valid = root.via.array.ptr[0].via.i64;
            FOR_N(i, n_channels)
               if (i < 11)
                  array[i] = root.via.array.ptr[i].via.dec;
            pthread_mutex_lock(&mutex);
            yaw = array[4];
            pitch = array[5];
            roll = array[6];
            pthread_mutex_unlock(&mutex);
         } 
         msgpack_unpacked_destroy(&msg);
      }
      else
      {
         msleep(1);
      }
   }
}


static int running = 1;
static msgpack_sbuffer *msgpack_buf = NULL;
static msgpack_packer *pk = NULL;

#define HIST_SZ 5
float hist_pitch[HIST_SZ] = {0.0, 0.0, 0.0, 0.0, 0.0};
float hist_roll[HIST_SZ] = {0.0, 0.0, 0.0, 0.0, 0.0};


float update_hist(float *hist, float val)
{
   for (int i = 0; i < HIST_SZ - 1; i++)
      hist[i] = hist[i + 1];
   hist[HIST_SZ - 1] = val;
   return hist[0];
}


int main(void)
{  
   cvNamedWindow("video");
   
   
   scl_init("object_tracking");
   void *obj_pos_socket = scl_get_socket("obj_pos");
   
   msgpack_buf = msgpack_sbuffer_new();
   pk = msgpack_packer_new(msgpack_buf, msgpack_sbuffer_write);
 
   pthread_t th;
   pthread_create(&th, NULL, thread, NULL);

    CvCapture* capture = 0;
    capture = cvCaptureFromCAM(0);
    if(!capture)
    {
        printf("Could not initialize capturing...");
        return -1;
    }
    while (1)
    {
        IplImage* frame = cvQueryFrame(capture);
        if (!frame)
        {
            msleep(1);
            continue;
        }
        IplImage* imgYellowThresh = GetThresholdedImage(frame);
        CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
        cvMoments(imgYellowThresh, moments, 1);
        double moment10 = cvGetSpatialMoment(moments, 1, 0);
        double moment01 = cvGetSpatialMoment(moments, 0, 1);
        double area = cvGetCentralMoment(moments, 0, 0);
        static int posX = 0;
        static int posY = 0;
        posX = moment10/area;
        posY = moment01/area;
        time_t t;
        t = clock();
        pthread_mutex_lock(&mutex);
        float lag_pitch = update_hist(hist_pitch, pitch);
        float lag_roll = update_hist(hist_pitch, roll);
        pthread_mutex_unlock(&mutex);
        
        float pos_x_norm = ((float)posX - 320.0f) / 640.0f;
        float pos_y_norm = ((float)posY - 240.0f) / 480.0f;
        float h = 0.3;
        float x = pos_x_norm * tan(45.0 * M_PI / 180.0) * 2.0 * h - tan(lag_pitch) * h;
        float y = pos_y_norm * tan(45.0 * M_PI / 180.0) * 2.0 * h + tan(lag_roll) * h;
        
        msgpack_sbuffer_clear(msgpack_buf);
        msgpack_pack_array(pk, 2);
        PACKF(x);
        PACKF(y);
        scl_copy_send_dynamic(obj_pos_socket, msgpack_buf->data, msgpack_buf->size);
 
        
        MyFilledCircle(frame, cvPoint(posX, posY));
        cvShowImage("video",frame);

        int c = cvWaitKey(33);
        if(27 == (char)c)
           break;
        
        cvReleaseImage(&imgYellowThresh);
        free(moments);
    }
    return 0;
}



