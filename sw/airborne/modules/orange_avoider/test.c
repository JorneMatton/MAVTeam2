#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav funcitons like Inside...
#include "generated/flight_plan.h"

float point_degree = 60.0;
float movex = 0.8947;
float movey = 0.4474;
int deg_acc = 1;
float boundary[2] = [-8.925, 8.925];
float margin = 0.5;
float slope = -6.8/3.2;

uint8_t point_to(float point_degree);
uint8_t between_lines(void);

void orange_avoider_periodic(void){
   float posx = stateGetPositionEnu_i()->x;
   float posy = stateGetPositionEnu_i()->y;
   float marker = posy + slope * posx;
   if ((leftboundary + margin) > (marker) || (marker) < (rightboundary - margin))
   {
      movex = -movex;
      movey = -movey;
      point_degree = point_degree + 180;
      VERBOSE_PRINT("Turned around")
}
