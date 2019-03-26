#include "modules/computer_vision/cv_detect_color_object.h"
#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"
#include "SURF_Integration.h"
#include "opencv_image_functions.h"

static pthread_mutex_t mutex;


typedef struct SURF_object {
    float * x_p;
    float * y_p;
}

struct SURF_object global_objects[1];   //variable that will be shared between video thread and autopilot thread

}

//Video callback
static struct image_t *surf_object_detector(struct image_t *img)
{
  //calculate object detection results 

  //convert the incoming image to grayscale
  Mat M(heigth, width, CV_8UC2, newImg);
  cvtColor(M, img, CV_YUV2GRAY_Y422);

  //now get the 
  vector<Keypoint> objects;
  objects = gesurfGetKeypointObjects(struct *img);

  pthread_mutex_lock(&mutex);
  global_object = objects;
  pthread_mutex_unlock(&mutex);

  return img;
}

//Surf module initialisation function
void surf_object_detector_init(void)
{
  cv_add_to_device(&COLORFILTER_CAMERA, surf_object_detector, COLORFILTER_FPS);
}


//Surf module periodic function (executed in the autopilotthread)
void surf_object_detector_periodic(void)
{
  static struct SURF_object local_objects

  pthread_mutex_lock(&mutex);  
  local_filter = global_filter;
  pthread_mutex_unlock(&mutex);

  AbiSendMsgSURF(SURF_OBJECT_DETECTION1_ID,local_object.x,local_object.y)
  
}
