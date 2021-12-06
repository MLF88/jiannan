#ifndef __TIGERH__
#define __TIGERH__

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define CLEAR(x) memset (&(x), 0, sizeof (x))
#define IMAGEWIDTH 1920
#define IMAGEHEIGHT 1080

using namespace cv;

struct buffer 
{
	void *start;
	size_t length;
};
/******function**********/
void init_device(char *camerapath);

Mat read_frame(Mat yuvImg, Mat bgrImg);

#endif
