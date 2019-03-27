#include "../computer_vision/colorfilter.h" /* needs modifying for proper building*/
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

struct image_t *colorfiler_func(struct image_t *img)
{
   return img;
}

void colorfilter_init(void)
{
   cv_add_to_device(&COLORFILTER_CAMERA, colorfiler_func, COLORFILTER_FPS);
}

int opencv(char *img, inf width, int height) {
   Mat M()
}
