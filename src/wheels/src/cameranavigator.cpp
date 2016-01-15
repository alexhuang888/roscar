#include "ros/ros.h"
#include "std_msgs/String.h"
#include "wheels/wheels_status.h"
#include "wheels/cmd_get_one_wheel_status.h"
#include "wheels/cmd_set_car_direction_speed.h"
#include "wheelcontroller.h"
#include "raspicam/raspicam_cv.h"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std; 

#define TESTROI 0
#define PRESERVEROI 1
#define THRESHOLDIMAGE 1

#define CAMERA_IMG_WIDTH 640
#define CAMERA_IMG_HEIGHT 480
#define IMGTHRESHOLD 70
#define MAXAREATHRESHOLD 2000
int32_t FindLineCenter(raspicam::RaspiCam_Cv &Camera, int32_t nTick, int32_t &nCenterX, int32_t &nCenterY)
{
	cv::Mat image;

	char szFilename[100] = "";
	char szOriFilename[100];
#if PRESERVEROI
	char szOriROIFilename[100];
	cv::Mat roiImgPreserve;
#endif

    cv::Rect roi(0, 190, CAMERA_IMG_WIDTH, 100);
    cv::Mat roiImg, erodeElmt, dilateElmt;
    int thVal = IMGTHRESHOLD;
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
 
	int nRet = -1;
	nCenterX = CAMERA_IMG_WIDTH / 2;
	nCenterY = CAMERA_IMG_HEIGHT / 2;
	
    {
		try {
			Camera.grab();
			Camera.retrieve(image);
			
			image(roi).copyTo(roiImg);
			#if PRESERVEROI
				roiImg.copyTo(roiImgPreserve);
			#endif
		} catch (cv::Exception& e)
		{
			const char* err_msg = e.what();
			std::cout << "exception caught: " << err_msg << std::endl;
		}
		
#if THRESHOLDIMAGE
		//printf("threshold image\n");
        cv::threshold(roiImg, roiImg, thVal, 255, cv::THRESH_BINARY);
        //printf("bitwise_not image\n");
        cv::bitwise_not(roiImg, roiImg); // negative image
        
		erodeElmt = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
		dilateElmt = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
		//printf("erode image\n");
		cv::erode(roiImg, roiImg, erodeElmt);
		//printf("dilate image\n");
		cv::dilate(roiImg, roiImg, dilateElmt); 
#endif 		
#if TESTROI		
		{
			sprintf(szFilename, "roi_image%d.png", nTick);
			sprintf(szOriFilename, "image%d.png", nTick);
			try {
				//cv::drawContours(roiImg, contours, s, cv::Scalar(255, 255, 255), 2, 8, hierarchy, 0, cv::Point() );
				cv::imwrite(szFilename, roiImg);
				cv::imwrite(szOriFilename, image);
			} catch (cv::Exception& e)
			{
				const char* err_msg = e.what();
				std::cout << "exception caught: " << err_msg << std::endl;
			}
		}	
#endif	
#if !TESTROI
		//printf("findContours image\n");
		cv::findContours(roiImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
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
		{
			if (fMaxArea > MAXAREATHRESHOLD && nMaxAreaContourIndex >= 0) 
			{
				cv::Moments mu;
				mu = cv::moments(contours[nMaxAreaContourIndex], false);
				cv::Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00); // point in center (x only)   
				printf("Find a region: Area=%f, center(%f, %f)\n", fMaxArea, center.x, center.y);
				
				nCenterX = (int32_t)center.x;
				nCenterY = (int32_t)center.y;
				nRet = 1;
				// here is the center pixel
#if 0
				{
					try {
						cv::circle(roiImg, center, 5, cv::Scalar(255, 255, 255), -1, 8, 0);
					} catch (cv::Exception& e)
					{
						const char* err_msg = e.what();
						std::cout << "exception caught: " << err_msg << std::endl;
					}
					sprintf(szFilename, "roi_image%d.png", nTick);
					sprintf(szOriFilename, "image%d.png", nTick);
					try {
						cv::drawContours(roiImg, contours, s, cv::Scalar(255, 255, 255), 2, 8, hierarchy, 0, cv::Point());
						cv::imwrite(szFilename, roiImg);
						//cv::imwrite(szOriFilename, image);
#if PRESERVEROI
						sprintf(szOriROIFilename, "ori_roi_image%d.png", nTick);
						cv::imwrite(szOriROIFilename, roiImgPreserve);
#endif						
					} catch (cv::Exception& e)
					{
						const char* err_msg = e.what();
						std::cout << "exception caught: " << err_msg << std::endl;
					}
				}
#endif				
			}
		}
#endif
    }
    
    return nRet;
}

int32_t FindOffset(int32_t nCenterX, int32_t nCenterY, float &fXOffset, float &fYOffset)
{
	int32_t nRet = 0;
	
	fXOffset = 0;
	fYOffset = 0;
	float nHalfWidth = CAMERA_IMG_WIDTH / 2.f;
	
	fXOffset = (nCenterX - nHalfWidth) / nHalfWidth;
	nRet = 1;
	
	return nRet;
}

int32_t OffsetNavigator(ros::ServiceClient &client, float fXOffset)
{
	int32_t nRet = 0;
	
	wheels::cmd_set_car_direction_speed srv2;
	
	
	if (fXOffset > 0)
		srv2.request.nNewDirection = 3;	// turn right
	else
		srv2.request.nNewDirection = 5;	// turn left
	
	srv2.request.nNewSpeed = 60;	// max speed;
		
	if (fabs(fXOffset) > 0.2 && fabs(fXOffset) < 0.4)	// If the offset is more than 30% on either side from the center of the image
	{
		srv2.request.nNewSpeed = 60;	// max speed;
	}
	else if(fabs(fXOffset) > 0.4 && fabs(fXOffset) < 0.6)	// If the offset is more than 50% on either side from the center of the image
	{
		srv2.request.nNewSpeed = 65;	// max speed;
	}
	else if(fabs(fXOffset) > 0.6 && fabs(fXOffset) < 0.8)	// If the offset is more than 70% on either side from the center of the image
	{
		srv2.request.nNewSpeed = 70;	// max speed;	
	}
	else if(fabs(fXOffset) > 0.8)	// If the offset is more than 90% on either side from the center of the image
	{
		srv2.request.nNewSpeed = 75;	// max speed;	
	}
	else	// Move forward with the specified speed by the user
	{
		srv2.request.nNewSpeed = 60;	// max speed;		
		srv2.request.nNewDirection = 1;	// forward
	}	
	if (client.call(srv2))
	{
		ROS_INFO("Set New Car direction=%d speed=%d", srv2.request.nNewDirection, srv2.request.nNewSpeed);
		ROS_INFO("Last car status: RetCode=%d: last_dir=%d, last_speed=%d", srv2.response.nRetCode, srv2.response.nLastDirection, srv2.response.nLastSpeed);
		nRet = 1;
	}
	else
	{
		ROS_ERROR("Failed to call service set_direction_speed");
	}
	return nRet;	
}
int main(int argc, char **argv)
{
	float fXOffset = 0, fYOffset = 0;
	int32_t nCenterX = 0, nCenterY = 0;
	uint32_t nTick = 0;
	
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
	ros::init(argc, argv, "camera_navigator");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
	ros::NodeHandle n;
	
	// setup camera
    raspicam::RaspiCam_Cv Camera;
    Camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);
	Camera.set(CV_CAP_PROP_FRAME_WIDTH, CAMERA_IMG_WIDTH);
    Camera.set(CV_CAP_PROP_FRAME_HEIGHT, CAMERA_IMG_HEIGHT);  
    cout << "Connecting to camera" << endl;
    
    if (!Camera.open())
	{
        cerr << "Error opening camera" << endl;
        return -1;
    }
    cout << "Connected to camera =" << Camera.getId() << endl;
	
	ros::ServiceClient client2 = n.serviceClient <wheels::cmd_set_car_direction_speed>("set_direction_speed");
	
	while (ros::ok())
	{
		FindLineCenter(Camera, nTick, nCenterX, nCenterY);
		
		FindOffset(nCenterX, nCenterY, fXOffset, fYOffset);
		
		OffsetNavigator(client2, fXOffset);
		
		nTick++;
	}

	Camera.release();
	return 1;
}
