/**
 * @file "modules/surf/surf_integration.c"
 * @author Jan Verheyen
 * This modules takes as input an image and applies the surf algorithm to detect objects hat are
 * coming close to the camera, given a certain treshold. The module sends information about 3 zones
 * (middle/2 side) to the orange_avoider.c module for further processing. This module is called 
 * by test1.c when an image becomes available from the camera.
 */
#ifndef SURF_INTEGRATION_H
#define SURF_INTEGRATION_H

#ifdef __cplusplus
extern "C" {
#endif


void surfDetectObjectsAndComputeControl(char *img, int imgWidth, int imgHeigth, int *zone1, int *zone2, int *zone3);

#ifdef __cplusplus
}
#endif

#endif