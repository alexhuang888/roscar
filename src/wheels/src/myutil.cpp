#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "stdarg.h"
CvPoint2D32f sub(CvPoint2D32f b, CvPoint2D32f a) { return cvPoint2D32f(b.x-a.x, b.y-a.y); }
CvPoint2D32f mul(CvPoint2D32f b, CvPoint2D32f a) { return cvPoint2D32f(b.x*a.x, b.y*a.y); }
CvPoint2D32f add(CvPoint2D32f b, CvPoint2D32f a) { return cvPoint2D32f(b.x+a.x, b.y+a.y); }
CvPoint2D32f mul(CvPoint2D32f b, float t) { return cvPoint2D32f(b.x*t, b.y*t); }
float dot(CvPoint2D32f a, CvPoint2D32f b) { return (b.x*a.x + b.y*a.y); }
float dist(CvPoint2D32f v) { return sqrtf(v.x*v.x + v.y*v.y); }

CvPoint2D32f point_on_segment(CvPoint2D32f line0, CvPoint2D32f line1, CvPoint2D32f pt)
{
	CvPoint2D32f v = sub(pt, line0);
	CvPoint2D32f dir = sub(line1, line0);
	float len = dist(dir);
	float inv = 1.0f/(len+1e-6f);
	dir.x *= inv;
	dir.y *= inv;

	float t = dot(dir, v);
	if (t >= len) return line1;
	else if(t <= 0) return line0;

	return add(line0, mul(dir,t));
}

float dist2line(CvPoint2D32f line0, CvPoint2D32f line1, CvPoint2D32f pt)
{
	return dist(sub(point_on_segment(line0, line1, pt), pt));
}
void crop(IplImage* src, IplImage* dest, CvRect rect)
{
	if (src != NULL && dest != NULL)
	{
		cvSetImageROI(src, rect);
		//printf("crop source (%d, %d), target(%d,%d)\n", src->width, src->height, dest->width, dest->height);
		cvCopy(src, dest);
		cvResetImageROI(src);
	}
}

double medianIplImage(IplImage* pSrc, int32_t nVals)
{
    // COMPUTE HISTOGRAM OF SINGLE CHANNEL MATRIX
    float range[] = { 0, (float)nVals };
    const float* histRange = { range };
    bool uniform = true;
    bool accumulate = false;
    cv::Mat hist;
    cv::Mat Input(pSrc);

    cv::calcHist(&Input, 1, 0, cv::Mat(), hist, 1, &nVals, &histRange, uniform, accumulate);

    // COMPUTE CUMULATIVE DISTRIBUTION FUNCTION (CDF)
    cv::Mat cdf;
    hist.copyTo(cdf);

    for (int i = 1; i <= nVals - 1; i++)
    {
        cdf.at<float>(i) += cdf.at<float>(i - 1);
    }
    cdf /= Input.total();

    // COMPUTE MEDIAN
    double medianVal;
    for (int i = 0; i <= nVals-1; i++)
    {
        if (cdf.at<float>(i) >= 0.5)
        {
            medianVal = i;
            break;
        }
    }
    return medianVal / nVals;
}
void myclearscreen(void)
{
    printf("\033[2J\n\033[0;0H");
}
void myprintf(int nRow, int nCol, const char *pFmt, ...)
{
    char szCode[100];
    char szAll[512];
    va_list args;
    va_start(args, pFmt);

    sprintf(szCode, "\033[%d;%dH\033[K\033[%d;%dH", nRow, nCol, nRow, nCol);
    printf("%s", szCode);

    //vprintf(pFmt, args);
    vsprintf(szAll, pFmt, args);
    printf("%s%s", szCode, szAll);
    va_end(args);
}

