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