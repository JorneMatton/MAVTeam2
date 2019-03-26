#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"
#include <stdio.h>

#include "pthread.h"
#include "surf_integration.h"
#include "test1.h"

static pthread_mutex_t mutex; 

#ifndef SURF_FPS
#define SURF_FPS 4       ///< Default FPS (zero means run at camera fps)
#endif

#define PRINT(string,...) fprintf(stderr, "[test1->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#define VERBOSE_PRINT PRINT


float global_heading_target; //variable that will be shared between video thread and autopilot thread

//Video callback
static struct image_t *surf_object_detector(struct image_t *img)
{
  float heading_target;

  surfDetectObjectsAndComputeControl((char *) img->buf, img->w, img->h, &heading_target);
  
  pthread_mutex_lock(&mutex);
  global_heading_target = heading_target;
  pthread_mutex_unlock(&mutex);

  return img;
}

//Surf module initialisation function
void surf_object_detector_init(void)
{
  cv_add_to_device(&COLOR_FILTER_CAMERA, surf_object_detector, SURF_FPS);
}


//Surf module periodic function (executed in the autopilotthread)
void surf_object_detector_periodic(void)
{
  float local_heading_target;

  pthread_mutex_lock(&mutex);  
  local_heading_target = global_heading_target;
  pthread_mutex_unlock(&mutex);

  VERBOSE_PRINT("Local heading target: %f \n", local_heading_target);

  AbiSendMsgSURF_OBSTACLE(SURF_OBJECT_DETECTION1_ID,local_heading_target);
  
}