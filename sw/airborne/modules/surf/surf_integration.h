#ifndef SURF_INTEGRATION_H
#define SURF_INTEGRATION_H

#ifdef __cplusplus
extern "C" {
#endif


void surfDetectObjectsAndComputeControl(char *img, int imgWidth, int imgHeigth, float *heading_target);

#ifdef __cplusplus
}
#endif

#endif