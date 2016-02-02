#include "cnavigatorenginebase.h"
#include <ros/callback_queue.h>

#include <boost/thread.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "cv_bridge/cv_bridge.h"

#include <iostream>
#include <fstream>
#include "globalinc.h"
#include "myutil.h"

#define _THISFILE_LINENO _APPLINENO_2
using namespace cv;
namespace yisys_roswheels
{
int32_t CNavigatorEngineBase::PublishDebugImage(std::string encoding, cv::Mat image)
{
    //if (pImage != NULL)
    {
        sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), encoding, image).toImageMsg();
        return PublishDebugImage(imgmsg);
    }

    return 0;
}

CNavigatorEngineWithImageSource::CNavigatorEngineWithImageSource()
{
	// subscribe
	m_strVidChannel = m_nImageBaseNodeHandle.resolveName("image");
	myprintf(_ERRORLINENO, 1, "Resolve name for image=%s\n", m_strVidChannel.c_str());

	m_VidSubscriber = m_nImageBaseNodeHandle.subscribe(m_strVidChannel, 1, &CNavigatorEngineWithImageSource::vidCb, this);


	// wait for cam calib
	m_nWidth = m_nHeight = 0;

	// imagebuffer
	//m_pImageBuffer = new NotifyBuffer<TimestampedMat>(8);
	//m_pUndistorter = NULL;
	//m_nLastSEQ = 0;

	m_bHaveCalibration = false;

	m_pNavEngineImpl = NULL;
	m_nLastTickCount = 0;
	m_nFrameProcessedCount = 0;
}

CNavigatorEngineWithImageSource::~CNavigatorEngineWithImageSource()
{
	//if (m_pUndistorter != NULL)
	//	delete m_pUndistorter;
	//m_pUndistorter = NULL;

	m_pNavEngineImpl = NULL;
}

int32_t CNavigatorEngineWithImageSource::SaveImage(uint32_t nModeFlags, std::string strImagePath)
{
    if (m_pNavEngineImpl != NULL)
    {
        return m_pNavEngineImpl->SaveImage(nModeFlags, strImagePath);
    }
    return 0;
}
int32_t CNavigatorEngineWithImageSource::SetNavEngineImplementation(CNavigatorEngineImplementationBase *pNavImpl)
{
    m_pNavEngineImpl = pNavImpl;
    if (m_pNavEngineImpl != NULL)
    {
        m_pNavEngineImpl->SetNavigatorEngineCB(this);
    }
    return 1;
}
uint32_t CNavigatorEngineWithImageSource::GetEngineID(void)
{
    if (m_pNavEngineImpl != NULL)
        return m_pNavEngineImpl->GetEngineID();

    return 0;
}
std::string CNavigatorEngineWithImageSource::GetEngineDescription(void)
{
    if (m_pNavEngineImpl != NULL)
        return m_pNavEngineImpl->GetEngineDescription();

    return "";
}
int32_t CNavigatorEngineWithImageSource::Init(void)
{
    if (m_pNavEngineImpl != NULL)
        return m_pNavEngineImpl->Init();

    return 0;
}
int32_t CNavigatorEngineWithImageSource::Start(void)
{
	m_nLastTickCount = cv::getTickCount();
	m_nFrameProcessedCount = 0;

    if (m_pNavEngineImpl != NULL)
        return m_pNavEngineImpl->Start();

    return 0;
}
int32_t CNavigatorEngineWithImageSource::Pause(void)
{
    if (m_pNavEngineImpl != NULL)
        return m_pNavEngineImpl->Pause();

    return 0;
}
bool CNavigatorEngineWithImageSource::IsPaused(void)
{
    if (m_pNavEngineImpl != NULL)
        return m_pNavEngineImpl->IsPaused();

    return true;
}
int32_t CNavigatorEngineWithImageSource::ProcessImageData(const sensor_msgs::ImageConstPtr img, bool bDisplayImage)
{
	int nRet = 0;
	float fAngle = 0.f;
	CvPoint vanishPoint;
	geometry_msgs::Twist vel_msg;
	float fAngleRatio = 0;

	SetDebugDisplayImage(bDisplayImage);// = bDisplayImage;

	if (IsPaused() == true)
	{
		nRet = 1;
		return nRet;
	}
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	//cv::Mat workingCVMat;
	IplImage WorkingImage = cv_ptr->image;//sensor_msgs::CvBridge::imgMsgToCv(img, "passthrough");
	/*
	if (m_pUndistorter != 0)
	{
		assert(m_pUndistorter->isValid());
		m_pUndistorter->undistort(cv_ptr->image, workingCVMat);
	}
*/
    if (m_pNavEngineImpl != NULL)
        nRet = m_pNavEngineImpl->ProcessImage(&WorkingImage, IsDebugDisplayImage(), fAngle, vanishPoint);
    else
        nRet = -100;

	if (nRet <= 0)
	{
		myprintf(_ERRORLINENO, 1, "Fail to ProcessImage\n");
		goto err_out;
	}
	//printf("Lane center: fAngle=%f, center(%d, %d)\n", fAngle, vanishPoint.x, vanishPoint.y);

	fAngleRatio = fAngle / 90.f;
#if 0
#define DIVRANGE 6.f

	fAngleRatio = ((int32_t)(fAngleRatio * DIVRANGE)) / DIVRANGE;
	myprintf(_ERRORLINENO, 1, "adjust angle=%f\n", fAngleRatio);
#endif
	vel_msg.angular.z = (fAngleRatio);
	vel_msg.linear.x = _CNEWIS_DEFAULT_LINEARSPEED;

	ProcessCmdVels(vel_msg);

err_out:
	return nRet;
}
int32_t CNavigatorEngineWithImageSource::setCalibration(std::string file)
{
	int32_t nRet = 0;

	if (file == "")
	{
		ros::Subscriber info_sub = m_nImageBaseNodeHandle.subscribe(m_nImageBaseNodeHandle.resolveName("camera_info"),
															1,
															&CNavigatorEngineWithImageSource::infoCb, this);

		printf("WAITING for ROS camera calibration!\n");
		while (m_nWidth == 0)
		{
			ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.03));
		}
		printf("RECEIVED ROS camera calibration!\n");

		info_sub.shutdown();

	}
	else
	{
		/*
		m_pUndistorter = Undistorter::getUndistorterForFile(file.c_str());

		if (m_pUndistorter == NULL)
		{
			printf("Failed to read camera calibration from file... wrong syntax?\n");
		}
		else
		{
			m_fx = m_pUndistorter->getK().at<double>(0, 0);
			m_fy = m_pUndistorter->getK().at<double>(1, 1);
			m_cx = m_pUndistorter->getK().at<double>(2, 0);
			m_cy = m_pUndistorter->getK().at<double>(2, 1);

			m_nWidth = m_pUndistorter->getOutputWidth();
			m_nHeight = m_pUndistorter->getOutputHeight();
		}
		*/
	}
	nRet = 1;
	m_bHaveCalibration = true;

err_out:
	return nRet;
}

// get called on ros-message callbacks
void CNavigatorEngineWithImageSource::vidCb(const sensor_msgs::ImageConstPtr img)
{
	#if 0
	if (!m_bHaveCalibration)
		return;
	#endif

	bool bDisplayImage = false;

	m_nImageBaseNodeHandle.param<bool>(WGP_DEBUG_SHOWIMAGE, bDisplayImage, false);
	//printf("Show Debug Image=%s\n", bDisplayImage ? "Yes" : "No");
	ProcessImageData(img, bDisplayImage);
#if 1
	{
        // find frame rate
        int64_t nThisTick = cv::getTickCount();
        double dSecond = (double)(nThisTick - m_nLastTickCount) / (double)cv::getTickFrequency();
        m_nFrameProcessedCount++;
        myprintf(_APPLINENO_4, 1, "Frame rate: %f (%d, %f)", m_nFrameProcessedCount / dSecond, m_nFrameProcessedCount, dSecond);
	}
#endif
}

void CNavigatorEngineWithImageSource::infoCb(const sensor_msgs::CameraInfoConstPtr info)
{
	if (!m_bHaveCalibration)
	{
		m_fx = info->P[0];
		m_fy = info->P[5];
		m_cx = info->P[2];
		m_cy = info->P[6];

		if (m_fx == 0 || m_fy == 0)
		{
			printf("camera calib from P seems wrong, trying calib from K\n");
			m_fx = info->K[0];
			m_fy = info->K[4];
			m_cx = info->K[2];
			m_cy = info->K[5];
		}

		m_nWidth = info->width;
		m_nHeight = info->height;

		printf("Received ROS Camera Calibration: fx: %f, fy: %f, cx: %f, cy: %f @ %dx%d\n",m_fx,m_fy,m_cx,m_cy,m_nWidth,m_nHeight);
	}
}
}
