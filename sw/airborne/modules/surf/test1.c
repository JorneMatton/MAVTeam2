#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"
#include <stdio.h>
#include <time.h>

#include "pthread.h"
#include "surf_integration.h"
#include "test1.h"

static pthread_mutex_t mutex; 

#ifndef SURF_FPS
#define SURF_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif

#define PRINT(string,...) fprintf(stderr, "[test1->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#define VERBOSE_PRINT PRINT


int global_zone1, global_zone2, global_zone3; //variables that will be shared between video thread and autopilot thread

//Video callback
static struct image_t *surf_object_detector(struct image_t *img)
{
  int zone1 = 0, zone2 = 0, zone3 = 0;

  clock_t tstart = clock();

  surfDetectObjectsAndComputeControl((char *) img->buf, img->w, img->h, &zone1, &zone2, &zone3);

  VERBOSE_PRINT("time taken: %.2fs \n", (double)(clock()-tstart)/CLOCKS_PER_SEC);
  
  pthread_mutex_lock(&mutex);
  global_zone1 = zone1;
  global_zone2 = zone2;
  global_zone3 = zone3;
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
  int local_zone1, local_zone2, local_zone3;


  pthread_mutex_lock(&mutex);  
  local_zone1 = global_zone1;
  local_zone2 = global_zone2;
  local_zone3 = global_zone3;
  pthread_mutex_unlock(&mutex);

  AbiSendMsgSURF_OBSTACLE(SURF_OBJECT_DETECTION1_ID, local_zone1, local_zone2, local_zone3);
  
}