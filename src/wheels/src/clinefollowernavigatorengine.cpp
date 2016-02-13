#include "iostream"
#include "vector"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "clinefollowernavigatorengine.h"
#include "myutil.h"
#define _THISFILE_LINENO _APPLINENO_2
namespace yisys_roswheels
{
	CLineFollowerNavigatorEngine::CLineFollowerNavigatorEngine()
	{
		m_bPaused = true;
	}

	CLineFollowerNavigatorEngine::~CLineFollowerNavigatorEngine()
	{
	}

	int32_t CLineFollowerNavigatorEngine::Init(void)
	{
		return 1;
	}

	int32_t CLineFollowerNavigatorEngine::Start(void)
	{
		m_bPaused = false;
		return 1;
	}

	int32_t CLineFollowerNavigatorEngine::Pause(void)
	{
		m_bPaused = true;
		return 1;
	}

//int32_t CLineFollowerNavigatorEngine::ProcessImageData(const sensor_msgs::ImageConstPtr img, bool bDisplayImage)
	int32_t CLineFollowerNavigatorEngine::ProcessImage(IplImage *pFrame, bool bDisplayImage, float &fAngle, CvPoint &vanishingPoint)
	{
		int nRet = 0;

		//SetDebugDisplayImage(bDisplayImage);

		if (m_bPaused == true)
		{
			nRet = 1;
			return nRet;
		}
		//cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
		cv::Mat workingCVMat;
		/*
		   if (m_pUndistorter != 0)
		   {
		    assert(m_pUndistorter->isValid());
		    m_pUndistorter->undistort(cv_ptr->image, workingCVMat);
		   }
		   else
		 */
		{
			workingCVMat = pFrame;
		}
		if (pFrame != NULL)
		{
			m_nWidth = pFrame->width;
			m_nHeight = pFrame->height;
		}
		//printf("workingCVMat: depth=%d, channels=%d, dims=%d, (%dx%d)\n", workingCVMat.depth(), workingCVMat.channels(), workingCVMat.dims, workingCVMat.rows, workingCVMat.cols);
		// process this gray-scale data
		int32_t nCenterX = 0, nCenterY = 0;
		float fOffsetX = 0, fOffsetY = 0;

		nRet = FindLineCenter(workingCVMat, 0, nCenterX, nCenterY, bDisplayImage);

		if (nRet <= 0)
		{
			//printf("Fail to find line center\n");
			goto err_out;
		}

		nRet = FindOffset(nCenterX, nCenterY, fOffsetX, fOffsetY);
		if (nRet <= 0)
		{
			myprintf(_ERRORLINENO, 1, "Fail to find offset\n");
			goto err_out;
		}

		nRet = OffsetNavigator(fOffsetX, fAngle, vanishingPoint);
		if (nRet <= 0)
		{
			myprintf(_ERRORLINENO, 1, "Fail to OffsetNavigator\n");
			goto err_out;
		}
err_out:
		return nRet;
	}
#define ROIRATAIO 4
	int32_t CLineFollowerNavigatorEngine::FindLineCenter(cv::Mat &InputImage, int32_t nTick, int32_t &nCenterX, int32_t &nCenterY, bool bDisplayImage)
	{
		char szFilename[100] = "";
		char szOriFilename[100];

#if PRESERVEROI
		char szOriROIFilename[100];
		cv::Mat roiImgPreserve;
#endif

		m_nHeight = InputImage.rows;
		m_nWidth = InputImage.cols;

		int nROILeft = 0;
		int nROIWidth = m_nWidth;
		int nROITop = m_nHeight * (ROIRATAIO - 1) / ROIRATAIO;
		int nROIHeight = m_nHeight / ROIRATAIO;

		cv::Rect roi(nROILeft, nROITop, nROIWidth, nROIHeight);
		cv::Mat roiImg, erodeElmt, dilateElmt;
		int thVal = IMGTHRESHOLD;
		std::vector < std::vector < cv::Point >> contours;
		std::vector < cv::Vec4i > hierarchy;

		int nRet = -1;
		int ContourAreaThreshold = roi.width * roi.height / 6;

		nCenterX = m_nWidth / 2;
		nCenterY = m_nHeight / 2;
		{
			try {
				InputImage(roi).copyTo(roiImg);
			#if PRESERVEROI
				if (bDisplayImage)
				{
					roiImg.copyTo(roiImgPreserve);
				}
			#endif
			} catch(cv::Exception & e)
			{
				const char* err_msg = e.what();
				std::cout << "exception caught: " << err_msg << std::endl;
			}

#if THRESHOLDIMAGE
			try {

				cv::cvtColor(roiImg, roiImg, CV_BGR2GRAY);
				cv::blur( roiImg, roiImg, cv::Size( 5, 5 ), cv::Point(-1, -1) );
				cv::threshold(roiImg, roiImg, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

			}
			catch(...)
			{
				myprintf(_ERRORLINENO, 1, "Fail to threhold image\n");
			}
#endif
#if TESTROI
			{
				sprintf(szFilename, "roi_image%d.png", nTick);
				sprintf(szOriFilename, "image%d.png", nTick);
				try {
					cv::imwrite(szFilename, roiImg);
					cv::imwrite(szOriFilename, image);
				} catch(cv::Exception & e)
				{
					const char* err_msg = e.what();
					std::cout << "exception caught: " << err_msg << std::endl;
				}
			}
#endif
#if !TESTROI
			//printf("findContours image\n");
			try {
				cv::findContours(roiImg, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
			} catch(...)
			{
				myprintf(_ERRORLINENO, 1, "Fail to findContours\n");
			}
			int nMaxAreaContourIndex = -1;
			float fMaxArea = 0.f;

			for (size_t s = 0; s < contours.size(); s++)
			{
				float fArea = cv::contourArea(contours[s]);
				if (fArea > fMaxArea)
				{
					nMaxAreaContourIndex = s;
					fMaxArea = fArea;
				}
			}
			myprintf(_THISFILE_LINENO, 1, "Found Max Area=%f, area threshold=%d (total contour=%d)\n", fMaxArea, ContourAreaThreshold, contours.size());
			{
				if (fMaxArea > ContourAreaThreshold && nMaxAreaContourIndex >= 0)
				{
					cv::Moments mu;
					try {
						mu = cv::moments(contours[nMaxAreaContourIndex], false);
					} catch(...)
					{
						myprintf(_ERRORLINENO, 1, "Fail to find moments\n");
					}
					cv::Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00); // point in center (x only)

					nCenterX = (int32_t)center.x + nROILeft;
					nCenterY = (int32_t)center.y + nROITop;
					nRet = 1;
					// here is the center pixel
					if (bDisplayImage)
					{
						try {
#if PRESERVEROI
							cv::circle(roiImgPreserve, center, 5, cv::Scalar(255, 255, 255), -1, 8, 0);
#else
							cv::circle(roiImg, center, 5, cv::Scalar(255, 255, 255), -1, 8, 0);
#endif
						} catch(cv::Exception & e)
						{
							const char* err_msg = e.what();
							std::cout << "exception caught: " << err_msg << std::endl;
						}
						try {
#if PRESERVEROI
							{
								PublishDebugImage("mono8", roiImgPreserve);
							}
#else
							{
								cv::drawContours(roiImg, contours, nMaxAreaContourIndex, cv::Scalar(255, 255, 255), 2, 8, hierarchy, 0, cv::Point());

								PublishDebugImage("mono8", roiImg);
							}
#endif

						} catch(cv::Exception & e)
						{
							const char* err_msg = e.what();
							std::cout << "exception caught: " << err_msg << std::endl;
						}
					}
					else
					{
						cv::destroyWindow("LineFollower");
					}

				}
			}
#endif
		}
err_out:
		return nRet;
	}

	int32_t CLineFollowerNavigatorEngine::FindOffset(int32_t nCenterX, int32_t nCenterY, float &fXOffset, float &fYOffset)
	{
		int32_t nRet = 0;

		fXOffset = 0;
		fYOffset = 0;
		float nHalfWidth = m_nWidth / 2.f;

		fXOffset = (nCenterX - nHalfWidth) / nHalfWidth;
		nRet = 1;

		return nRet;
	}

	int32_t CLineFollowerNavigatorEngine::OffsetNavigator(float fXOffset, float &fAngle, CvPoint &vanishingPoint)
	{
		int32_t nRet = 0;

		fAngle = fXOffset * 90.;

		nRet = 1;
		return nRet;
	}

}
