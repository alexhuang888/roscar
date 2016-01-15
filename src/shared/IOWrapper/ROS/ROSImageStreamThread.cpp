/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ROSImageStreamThread.h"
#include <ros/callback_queue.h>

#include <boost/thread.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "cv_bridge/cv_bridge.h"
#include "util/settings.h"

#include <iostream>
#include <fstream>


namespace yisys_rosshared
{


using namespace cv;

ROSImageStreamThread::ROSImageStreamThread()
{
	// subscribe
	m_strVidChannel = m_NodeHandle.resolveName("image");
	m_VidSubscriber          = m_NodeHandle.subscribe(m_strVidChannel,1, &ROSImageStreamThread::vidCb, this);


	// wait for cam calib
	m_nWidth = m_nHeight = 0;

	// imagebuffer
	m_pImageBuffer = new NotifyBuffer<TimestampedMat>(8);
	m_pUndistorter = 0;
	m_nLastSEQ = 0;

	m_bHaveCalibration = false;
}

ROSImageStreamThread::~ROSImageStreamThread()
{
	delete m_pImageBuffer;
}

void ROSImageStreamThread::setCalibration(std::string file)
{
	if(file == "")
	{
		ros::Subscriber info_sub         = m_NodeHandle.subscribe(m_NodeHandle.resolveName("camera_info"),1, &ROSImageStreamThread::infoCb, this);

		printf("WAITING for ROS camera calibration!\n");
		while(m_nWidth == 0)
		{
			ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.03));
		}
		printf("RECEIVED ROS camera calibration!\n");

		info_sub.shutdown();
	}
	else
	{
		m_pUndistorter = Undistorter::getUndistorterForFile(file.c_str());

		if(m_pUndistorter==0)
		{
			printf("Failed to read camera calibration from file... wrong syntax?\n");
			exit(0);
		}

		m_fx = m_pUndistorter->getK().at<double>(0, 0);
		m_fy = m_pUndistorter->getK().at<double>(1, 1);
		m_cx = m_pUndistorter->getK().at<double>(2, 0);
		m_cy = m_pUndistorter->getK().at<double>(2, 1);

		m_nWidth = m_pUndistorter->getOutputWidth();
		m_nHeight = m_pUndistorter->getOutputHeight();
	}

	m_bHaveCalibration = true;
}

void ROSImageStreamThread::run()
{
	boost::thread thread(boost::ref(*this));
}

void ROSImageStreamThread::operator()()
{
	ros::spin();

	exit(0);
}


void ROSImageStreamThread::vidCb(const sensor_msgs::ImageConstPtr img)
{
	if(!m_bHaveCalibration) return;

	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

	if(img->header.seq < (unsigned int)m_nLastSEQ)
	{
		printf("Backward-Jump in SEQ detected, but ignoring for now.\n");
		m_nLastSEQ = 0;
		return;
	}
	m_nLastSEQ = img->header.seq;

	TimestampedMat bufferItem;
	if(img->header.stamp.toSec() != 0)
		bufferItem.timestamp =  Timestamp(img->header.stamp.toSec());
	else
		bufferItem.timestamp =  Timestamp(ros::Time::now().toSec());

	if(m_pUndistorter != 0)
	{
		assert(m_pUndistorter->isValid());
		m_pUndistorter->undistort(cv_ptr->image,bufferItem.data);
	}
	else
	{
		bufferItem.data = cv_ptr->image;
	}

	m_pImageBuffer->pushBack(bufferItem);
}

void ROSImageStreamThread::infoCb(const sensor_msgs::CameraInfoConstPtr info)
{
	if(!m_bHaveCalibration)
	{
		m_fx = info->P[0];
		m_fy = info->P[5];
		m_cx = info->P[2];
		m_cy = info->P[6];

		if(m_fx == 0 || m_fy==0)
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
