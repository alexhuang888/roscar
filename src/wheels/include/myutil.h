#ifndef __MYUTIL_H__
#define __MYUTIL_H__
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
CvPoint2D32f sub(CvPoint2D32f b, CvPoint2D32f a);
CvPoint2D32f mul(CvPoint2D32f b, CvPoint2D32f a);
CvPoint2D32f add(CvPoint2D32f b, CvPoint2D32f a);
CvPoint2D32f mul(CvPoint2D32f b, float t);
float dot(CvPoint2D32f a, CvPoint2D32f b);
float dist(CvPoint2D32f v);

CvPoint2D32f point_on_segment(CvPoint2D32f line0, CvPoint2D32f line1, CvPoint2D32f pt);

float dist2line(CvPoint2D32f line0, CvPoint2D32f line1, CvPoint2D32f pt);

void crop(IplImage* src, IplImage* dest, CvRect rect);

double medianIplImage(IplImage* src, int32_t nVals);

void myprintf(int nRow, int nCol, const char *pFmt, ...);
void myclearscreen(void);


#define _APPLINENO_0 3
#define _APPLINENO_1 5
#define _APPLINENO_2 7
#define _APPLINENO_3 9
// here is for main app use
#define _APPLINENO_4 13
#define _APPLINENO_5 17
#define _ERROR_LINENO 25
#define _ERRORLINENO _ERROR_LINENO
#define _INFOLINENO 21

#endif
