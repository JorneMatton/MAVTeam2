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


#define NAV_C // needed to get the nav funcitons like Inside...
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
uint8_t moveWaypointSideways(uint8_t waypoint);
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
uint8_t increase_nav_heading(float incrementDegrees);
uint8_t chooseRandomIncrementAvoidance(void);
uint8_t chooseIncrementAvoidance(void); 
int largest(int arr[], int n);
int smallest(int arr[], int n);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};

// define settings
//float oa_color_count_frac = 0.18f;

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
std::vector <int16_t> x,y;               // list of x values of obstacles
int16_t obstacle_free_confidence = 0; // a measure of how certain we are that the way ahead is safe.
float heading_increment = 5.f;          // heading angle increment [deg]
float maxDistance = 2.25;               // max waypoint displacement [m]
int ROIw=1000;// number of pixels in image width (TO BE DEFINED) 

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

#ifndef ORANGE_AVOIDER_SURF_OBSTACLE_ID
#define ORANGE_AVOIDER_SURF_OBSTACLE_ID ABI_BROADCAST
#endif
static abi_event surf_detection_ev;
static void surf_detection_cb(uint8_t sender_id,int16t x_obstacle, y_obstacle)
{
	
 for( int i = 0; i < x_obstacle.size(); i++){
	 x.push_back(x_obstacle[i]);
	 y.push_back(y_obstacle[i]);
 }
  x = x_obstacle;
  y = y_obstacle;
  
}

/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void orange_avoider_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgSURF_OBSTACLE(ORANGE_AVOIDER_SURF_OBSTACLE_ID, &surf_detection_ev, surf_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void orange_avoider_periodic(void)
{
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }

  // compute current color thresholds
  //int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

  //VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count, navigation_state);

  // update our safe confidence using color threshold
  if(x.empty()){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);

  switch (navigation_state){
    case SAFE:
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
      }

      break;
    case OBSTACLE_FOUND:
      //stop
		waypoint_set_here_2d(WP_GOAL);
		waypoint_set_here_2d(WP_TRAJECTORY);
    	//moveWaypointSideways(WP_TRAJECTORY);
    	//moveWaypointSideways(WP_GOAL);
      // inteligently select new search direction
		chooseIncrementAvoidance();
		if (obstacle_free_confidence >= 2){
        navigation_state = SAFE;
      }
      break;
      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:
      increase_nav_heading(heading_increment);

      // make sure we have a couple of good readings before declaring the way safe
      if (obstacle_free_confidence >= 2){
        navigation_state = SAFE;
      }
      break;
    case OUT_OF_BOUNDS:
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        increase_nav_heading(heading_increment);

        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      break;
    default:
      break;
  }
  return;
}


/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  VERBOSE_PRINT("distance is%f", distanceMeters);
  return false;
 
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' to the right! w.r.t. current position and heading
 */
static uint8_t calculateSideways(struct EnuCoor_i *new_coor)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x =  stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(cosf(heading) * (1));
  new_coor->y =  stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(sinf(heading) * (1));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}
/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}
// this moves the waypoint to the right
uint8_t moveWaypointSideways(uint8_t waypoint)
{
  struct EnuCoor_i new_coor;
  calculateSideways(&new_coor);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

int largest(int arr[], int n) 
{ 
    int i; 
     
    // Initialize maximum element 
    int max = arr[0]; 
  
    // Traverse array elements from second and 
    // compare every element with current max   
    for (i = 1; i < n; i++) 
        if (arr[i] > max) 
            max = arr[i]; 
  
    return max; 
} 
int smallest(int arr[], int n) 
{ 
    int i; 
     
    // Initialize maximum element 
    int max = arr[0]; 
  
    // Traverse array elements from second and 
    // compare every element with current max   
    for (i = 1; i < n; i++) 
        if (arr[i] < min) 
            min = arr[i]; 
  
    return min; 
} 
/*
 * Sets the variable 'heading_increment' positive/negative
 */
uint8_t chooseIncrementAvoidance(void)
{
int n = sizeof(x)/sizeof(x[0]); 
int right_X=largest(x,n);
int down_Y=smallest(y,n);
int left_X=smallest(x,n);
int upper_Y=largest(y,n);
int area_L=abs(-ROIw/2-left_X)*abs(upper_Y-down_Y);
int area_R=abs(ROIw/2-right_X)*abs(upper_Y-down_Y);
  // chooses CW or CCW according to image
  if (area_R>area_L) {
    heading_increment = 5.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
    heading_increment = -5.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  return false;
}


/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = 5.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
    heading_increment = -5.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  return false;
}
