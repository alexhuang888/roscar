#ifndef __CNAVIGATORENGINEIMPLEMENTATION_H__
#define __CNAVIGATORENGINEIMPLEMENTATION_H__
#include "opencv2/imgproc/imgproc.hpp"
namespace yisys_roswheels
{
class CNavigatorEngineCB
{
public:
    virtual int32_t PublishDebugImage(std::string encoding, cv::Mat image) = 0;
};
class CNavigatorEngineImplementationBase
{
public:
    virtual int32_t Init(void) = 0;
	virtual int32_t Start(void) = 0;
	virtual int32_t Pause(void) = 0;
	virtual bool IsPaused(void) = 0;
    // process image
    virtual int32_t ProcessImage(IplImage *pFrame, bool bDisplayImage, float &fAngle, CvPoint &vanishingPoint) = 0;
	virtual uint32_t GetEngineID(void) = 0;
	virtual std::string GetEngineDescription(void) = 0;

	virtual int32_t SetNavigatorEngineCB(CNavigatorEngineCB *pCB) { m_pNavEngineCB = pCB; return 1; }

	virtual int32_t PublishDebugImage(std::string encoding, cv::Mat image)
	{
        if (m_pNavEngineCB != NULL)
            return m_pNavEngineCB->PublishDebugImage(encoding, image);
        return 0;
    }
    virtual int32_t SaveImage(uint32_t nModeFlags, std::string strImagePath) {return 0;};
protected:
    CNavigatorEngineCB *m_pNavEngineCB;
};
};

#endif
