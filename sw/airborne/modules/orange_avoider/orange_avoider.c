/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the navigation mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 */

#include "modules/orange_avoider/orange_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <time.h>
#include <stdio.h>
#include "std.h"
#include "subsystems/datalink/telemetry.h"



#define NAV_C // needed to get the nav funcitons like Inside...
#include "generated/flight_plan.h"

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#define VERBOSE_PRINT PRINT

#ifndef ORANGE_AVOIDER_LUM_MIN
#define ORANGE_AVOIDER_LUM_MIN 41
#endif

#ifndef ORANGE_AVOIDER_LUM_MAX
#define ORANGE_AVOIDER_LUM_MAX 183
#endif

#ifndef ORANGE_AVOIDER_CB_MIN
#define ORANGE_AVOIDER_CB_MIN 53
#endif

#ifndef ORANGE_AVOIDER_CB_MAX
#define ORANGE_AVOIDER_CB_MAX 121
#endif

#ifndef ORANGE_AVOIDER_CR_MIN
#define ORANGE_AVOIDER_CR_MIN 134
#endif

#ifndef ORANGE_AVOIDER_CR_MAX
#define ORANGE_AVOIDER_CR_MAX 249
#endif

#define AMOUNT_LANES 9

int badlanes[AMOUNT_LANES] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
float point_degree = 60.0;
float movex = 6.8/7.60;
float movey = 3.4/7.60;
int deg_acc = 1;
float leftboundary = -8.925;
float rightboundary =  8.925;
float margin = 4.0;
float slope = 6.8/3.2;
int counter = 0;
float error_b = 0.0;
int lane_b = 0;
int lane_gain = 2;
float lane_confidence = 0.25;



uint8_t moveWaypointForward(uint8_t waypoint, float movex, float movey, float error_b);
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
// uint8_t increase_nav_heading(float incrementDegrees);
// uint8_t chooseRandomIncrementAvoidance(void);
uint8_t chooseIncrementAvoidance(void);
uint8_t point_to(float point_degree);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};

// define settings

float oa_color_count_frac = 0.12f;



// define and initialise global variables
enum navigation_state_t navigation_state = SAFE;
int obstacle_free_confidence = 0; // a measure of how certain we are that the way ahead is safe.
float heading_increment = 5.f;          // heading angle increment [deg]
float maxDistance = 2.25;               // max waypoint displacement [m]
int32_t color_count = 0; // orange color count from color filter for obstacle detection

typedef unsigned short uint16;

int zone_left,zone_middle,zone_right;              // the three detection zones where the keypoints determined by surf are counted
const int treshold_left = 10;                  // treshold values for the detection zones
const int treshold_middle = 9;
const int treshold_right = 10;

const int16_t max_trajectory_confidence = 10; // number of consecutive negative object detections to be sure we are obstacle free

/*
 * ABI stuff
 */
#ifndef ORANGE_AVOIDER_SURF_OBSTACLE_ID
#define ORANGE_AVOIDER_SURF_OBSTACLE_ID ABI_BROADCAST
#endif
static abi_event surf_detection_ev;
static void surf_detection_cb(uint8_t __attribute__((unused)) sender_id, int zone1, int zone2, int zone3)
{
zone_left = zone1;
zone_middle = zone2;
zone_right = zone3;
}



#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count = quality;
}





static void send_orange_avoider_data(struct transport_tx *trans , struct link_device *dev){
  uint16 a = (uint16)zone_left, b = (uint16)zone_middle,c = (uint16)zone_right, d = (uint16)obstacle_free_confidence, e = (uint16)navigation_state;
  pprz_msg_send_ORANGE_AVOIDER(trans,dev,AC_ID,&a,&b,&c,&d,&e);
}

/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void orange_avoider_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseIncrementAvoidance();

  // bind our colorfilter and SURF algorithm callbacks to receive the color filter and SURF outputs
  AbiBindMsgSURF_OBSTACLE(ORANGE_AVOIDER_SURF_OBSTACLE_ID, &surf_detection_ev, surf_detection_cb);
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  register_periodic_telemetry(DefaultPeriodic,PPRZ_MSG_ID_ORANGE_AVOIDER,send_orange_avoider_data);
}

/*
 * Function that checks if it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void orange_avoider_periodic(void)
{
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }

  int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

  VERBOSE_PRINT("color count: %d, zone1: %d, zone2: %d, zone3: %d,  obstacle free confidence: %d\n", color_count, zone_left, zone_middle, zone_right, obstacle_free_confidence);
  // update our safe confidence using heading_target input (from SURF feature detection)
  if(zone_middle<treshold_middle && color_count < color_count_threshold){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 3;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float moveDistance = fminf(maxDistance, 0.1f * obstacle_free_confidence);

  switch (navigation_state){
    case SAFE:
      if (stateGetNedToBodyEulers_f()->psi > RadOfDeg(point_degree + deg_acc) || stateGetNedToBodyEulers_f()->psi < RadOfDeg(point_degree - deg_acc)){
         point_to(point_degree);
        }
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, movex, movey, error_b);
      moveWaypointForward(WP_GOAL, movex, movey, error_b);
            float posx = POS_FLOAT_OF_BFP(stateGetPositionEnu_i()->x);
      float posy = POS_FLOAT_OF_BFP(stateGetPositionEnu_i()->y);
      float marker = posy + slope * posx;
      error_b = lane_gain * (lane_b - posy + movey/movex * posx);
      VERBOSE_PRINT("Margin: %f.Marker: %f. error_b: %f\n", margin, marker, error_b);
      VERBOSE_PRINT("lane_b = %d, error_b = %f\n", lane_b, error_b);

      if ((leftboundary + margin) > (marker) || (marker) > (rightboundary - margin)){
         counter++;
         if (counter == 1)
         {
            movex = -movex;
            movey = -movey;
            point_degree = point_degree + 180;
            VERBOSE_PRINT("Turn around\n");
          }
      }
      else if (obstacle_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
      } else {
        counter = 0;
      }
      break;

    case OBSTACLE_FOUND:

		  chooseIncrementAvoidance();
      obstacle_free_confidence = 4;    
      navigation_state = SAFE;
      break;

    default:
      break;
  }
  return;
}


/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
// uint8_t increase_nav_heading(float incrementDegrees)
// {
//   float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

//   // normalize heading to [-pi, pi]
//   FLOAT_ANGLE_NORMALIZE(new_heading);

//   // set heading
//   nav_heading = ANGLE_BFP_OF_REAL(new_heading);

//   // VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
//   return false;
// }

uint8_t point_to(float point_degree)
{
   float new_heading = RadOfDeg(point_degree);

   // normalize heading to [-pi, pi]
   FLOAT_ANGLE_NORMALIZE(new_heading);

   // set heading
   nav_heading = ANGLE_BFP_OF_REAL(new_heading);

   VERBOSE_PRINT("Set heading to %f\n", DegOfRad(new_heading));
   return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float movex, float movey, float error_b)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(movex);
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(movey + error_b);
  // VERBOSE_PRINT("point: x: %f,  y: %f based on pos(%f, %f) and heading(%f) with %f\n", POS_FLOAT_OF_BFP(new_coor->x),  POS_FLOAT_OF_BFP(new_coor->y), stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading), POS_BFP_OF_REAL(movex));
return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  // VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                // POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float movex, float movey, float error_b)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, movex, movey, error_b);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Sets the variable 'heading_increment' positive/negative
 */
uint8_t chooseIncrementAvoidance(void)
{
  if ((error_b*error_b < lane_confidence * lane_confidence * lane_gain * lane_gain) && (zone_left < treshold_left || zone_right < treshold_right)){
     badlanes[lane_b] = 1;
    if(zone_left<zone_right){
      if (movex > 0){ // flying to the right
         lane_b++;
      } else{
         lane_b--;
      }
    }
    else if(zone_left>zone_right){
      if (movex > 0){
         lane_b--;
      } else{
         lane_b++;
      }
    } 
    else if (lane_b > 3){
      lane_b = 1;
      }
    else if(lane_b < -3){
      lane_b = -1;
      }
  }
  else{
     movex = -movex;
     movey = -movey;
     point_degree = point_degree + 180;
     VERBOSE_PRINT("Turn around");
  }
  return false;
}

// /*
//  * Sets the variable 'heading_increment' randomly positive/negative
//  */
// uint8_t chooseRandomIncrementAvoidance(void)
// {
//   // Randomly choose CW or CCW avoiding direction
//   if (rand() % 2 == 0) {
//     heading_increment = 15.f;
//     VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
//   } else {
//     heading_increment = -15.f;
//     VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
//   }
//   return false;
// }
