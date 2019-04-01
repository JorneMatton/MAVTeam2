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
#ifndef TEST1_H
#define TEST1_H


extern void surf_object_detector_init(void);
extern void surf_object_detector_periodic(void);

#endif