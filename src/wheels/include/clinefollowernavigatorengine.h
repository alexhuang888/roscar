#ifndef __CLINEFOLLOWERNAVIGATORENGINE_H__
#define __CLINEFOLLOWERNAVIGATORENGINE_H__

#pragma once

#include "opencv2/imgproc/imgproc.hpp"
#include "cnavigatorengineimplementationbase.h"
namespace yisys_roswheels
{
/*
 * in this class, it will get camera data from image_raw message, then process it to get estimated speed and direction
 * for the wheel
 */
#define TESTROI 0
#define PRESERVEROI 0
#define THRESHOLDIMAGE 1

#define IMGTHRESHOLD 127


#define WEID_LINEFOLLOWERENGINE 1
#define WESTR_LINEFOLLOWERENGINE "Line Follower"

class CLineFollowerNavigatorEngine : public CNavigatorEngineImplementationBase
{
public:
	CLineFollowerNavigatorEngine();
	virtual ~CLineFollowerNavigatorEngine();

	virtual int32_t Init(void);
	virtual int32_t Start(void);
	virtual int32_t Pause(void);
    virtual bool IsPaused(void) { return m_bPaused; }
	//virtual int32_t ProcessImageData(const sensor_msgs::ImageConstPtr img, bool bDisplayImage);
    virtual int32_t ProcessImage(IplImage *pFrame, bool bDisplayImage, float &fAngle, CvPoint &vanishingPoint);
	virtual uint32_t GetEngineID(void) { return WEID_LINEFOLLOWERENGINE; };
	virtual std::string GetEngineDescription(void) { return WESTR_LINEFOLLOWERENGINE;} ;
protected:
	int32_t FindLineCenter(cv::Mat &InputImage, int32_t nTick, int32_t &nCenterX, int32_t &nCenterY, bool bDisplayImage);
	int32_t FindOffset(int32_t nCenterX, int32_t nCenterY, float &fXOffset, float &fYOffset);
	int32_t OffsetNavigator(float fXOffset, float &fAngle, CvPoint &vanishingPoint);
protected:

	bool m_bPaused;
	int32_t m_nWidth, m_nHeight;
};
};
#endif
