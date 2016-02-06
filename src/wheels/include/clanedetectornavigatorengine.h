#ifndef __CLANEDETECTORNAVIGATORENGINE_H__
#define __CLANEDETECTORNAVIGATORENGINE_H__

#pragma once
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <list>
#include "myutil.h"
#include "claneinfo.h"
#include "cnavigatorengineimplementationbase.h"
namespace yisys_roswheels
{
/*
 * in this class, it will get camera data from image_raw message, then process it to get estimated speed and direction
 * for the wheel
 */
#define DEBUGIMG 0

#define K_VARY_FACTOR 0.2f
#define B_VARY_FACTOR 20
#define MAX_LOST_FRAMES 30

#define WEID_LANEDETECTORENGINE 2
#define WESTR_LANEDETECTORENGINE "Lane Detector"



enum{
    SCAN_STEP = 5,			  // in pixels
	LINE_REJECT_DEGREES = 10, // in degrees
    BW_TRESHOLD = 250,		  // edge response strength to recognize for 'WHITE'
    BORDERX = 10,			  // px, skip this much from left & right borders
	MAX_RESPONSE_DIST = 5,	  // px

	CANNY_MIN_TRESHOLD = 30,	  // edge detector minimum hysteresis threshold
	CANNY_MAX_TRESHOLD = 150, // edge detector maximum hysteresis threshold

	HOUGH_TRESHOLD = 50,		// line approval vote threshold
	HOUGH_MIN_LINE_LENGTH = 50,	// remove lines shorter than this treshold
	HOUGH_MAX_LINE_GAP = 100,   // join lines to one with smaller than this gaps

	CAR_DETECT_LINES = 4,    // minimum lines for a region to pass validation as a 'CAR'
	CAR_H_LINE_LENGTH = 10,  // minimum horizontal line length from car body in px

	MAX_VEHICLE_SAMPLES = 30,      // max vehicle detection sampling history
	CAR_DETECT_POSITIVE_SAMPLES = MAX_VEHICLE_SAMPLES-2, // probability positive matches for valid car
	MAX_VEHICLE_NO_UPDATE_FREQ = 15 // remove car after this much no update frames
};



class CLaneDetectorNavigatorEngine : public CNavigatorEngineImplementationBase
{
public:
	CLaneDetectorNavigatorEngine();
	virtual ~CLaneDetectorNavigatorEngine();

	virtual int32_t Init(void);
	virtual int32_t Start(void);
	virtual int32_t Pause(void);
    virtual bool IsPaused(void) { return m_bPaused; }
	//virtual int32_t ProcessImageData(const sensor_msgs::ImageConstPtr img, bool bDisplayImage);
    virtual int32_t ProcessImage(IplImage *pFrame, bool bDisplayImage, float &fAngle, CvPoint &vanishingPoint);
	virtual uint32_t GetEngineID(void) { return WEID_LANEDETECTORENGINE; };
	virtual std::string GetEngineDescription(void) { return WESTR_LANEDETECTORENGINE;} ;
protected:
	int32_t ProcessFrame(IplImage *pImage, float &fAngle, CvPoint &vanishingPoint);
	CLaneStatus &GetRightLane(void) { return m_LaneR; }
	CLaneStatus &GetLeftLane(void) { return m_LaneL; }
	void ShowLine(bool bShow) { m_bShowLine = bShow; }
protected:
	void FindResponses(IplImage *img, int startX, int endX, int y, std::vector<int>& list);
	void ProcessSide(std::vector<CLaneInfo> lanes, IplImage *edges, bool right);
	void ProcessLanes(CvSeq* lines, IplImage* edges, IplImage* temp_frame, bool bShowHoughLine, bool bDisplayImage);
protected:
	// left and right lane
	CLaneStatus m_LaneR, m_LaneL;
	CvMemStorage* m_pHoughStorage;
	CvSize m_FrameSize;
	CvSize m_HalfFrameSize;
	CvRect m_ROI;
	CvFont m_Font;
	IplImage *m_pWorkingImage, *m_pGreyImage, *m_pEdgesImage;
	bool m_bShowLine;
	float m_fTurnAngle;
	CvPoint m_VanishingPoint;

protected:

	bool m_bPaused;
};
};
#endif
