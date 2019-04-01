/*
 * Copyright (C) Mav Team 2
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Jan Verheyen
 * This module takes inputs from both the colorfilter (cv_detect_color_object.c) and the SURF algorithm (surf_integration.c)
 * and is combined with the navigation mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn. In addition to this, a SURF algorithm is 
 * utilised to detect more complex objects that preferably have a texture, the amount of objects in 3 different zones is
 * counted and when the middle zone goes over the treshold the control algorithm takes appropriate measure.
 */

#ifndef ORANGE_AVOIDER_H
#define ORANGE_AVOIDER_H

// settings
extern float oa_color_count_frac;

// functions
extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);

#endif

