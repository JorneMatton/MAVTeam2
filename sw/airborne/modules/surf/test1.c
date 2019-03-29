/*
 * Copyright (C) Jip
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/surf/test1.c"
 * @author Jan Verheyen
 * This is the main running module of the SURF algorithm.
 * This module binds the images received from the cv.c module and sends them to surf_integration.cpp, where the surf algorithm is run.
 * Furthermore, it communicates with the orange_avoider module to send the appropriate values
 */

#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"
#include <stdio.h>
#include <time.h>
#include "subsystems/datalink/telemetry.h"

#include "pthread.h"
#include "surf_integration.h"
#include "test1.h"

static pthread_mutex_t mutex; 

#ifndef SURF_FPS
#define SURF_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif

#define PRINT(string,...) fprintf(stderr, "[test1->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#define VERBOSE_PRINT PRINT


int global_zone1, global_zone2, global_zone3; // variables that will be shared between video thread and autopilot thread
float global_timediff;                        // used for benchmarking the SURF algorithm

//Video callback
static struct image_t *surf_object_detector(struct image_t *img)
{
  // initialise count of the 3 detection zones
  int zone1 = 0, zone2 = 0, zone3 = 0;

  clock_t tstart = clock();
  // send image and the to be send zones with the number of objects to the surf algorithm
  surfDetectObjectsAndComputeControl((char *) img->buf, img->w, img->h, &zone1, &zone2, &zone3);
  float timediff = (float)(clock()-tstart)/CLOCKS_PER_SEC;

  pthread_mutex_lock(&mutex);
  global_timediff = timediff;
  global_zone1 = zone1;
  global_zone2 = zone2;
  global_zone3 = zone3;
  pthread_mutex_unlock(&mutex);

  return img;
}

static void send_surf_data(struct transport_tx *trans , struct link_device *dev){
  pprz_msg_send_SURF(trans,dev,AC_ID, &global_timediff);
}

//Surf module initialisation function
void surf_object_detector_init(void)
{
  // subscribe to images from the camera, run callback function when image received
  cv_add_to_device(&COLOR_FILTER_CAMERA, surf_object_detector, SURF_FPS);
  // bind telemetry callback to send the desired variables
  register_periodic_telemetry(DefaultPeriodic,PPRZ_MSG_ID_SURF,send_surf_data);
}


//Surf module periodic function
void surf_object_detector_periodic(void)
{
  int local_zone1, local_zone2, local_zone3;

  pthread_mutex_lock(&mutex);  
  local_zone1 = global_zone1;
  local_zone2 = global_zone2;
  local_zone3 = global_zone3;
  pthread_mutex_unlock(&mutex);
  // sent number of objects detected in the 3 detection zones
  AbiSendMsgSURF_OBSTACLE(SURF_OBJECT_DETECTION1_ID, local_zone1, local_zone2, local_zone3);
  
}