#ifndef __ARUCOMARKERNAGIVATOR_H__
#define __ARUCOMARKERNAGIVATOR_H__
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "wheels/wheels_status.h"
#include "wheels/cmd_get_one_wheel_status.h"
#include "wheels/cmd_set_car_direction_speed.h"
#include <geometry_msgs/Twist.h>

#pragma once
namespace yisys_roswheels
{
#define TESTROI 0
#define PRESERVEROI 1
#define THRESHOLDIMAGE 1

#define CAMERA_IMG_WIDTH 640
#define CAMERA_IMG_HEIGHT 480
#define IMGTHRESHOLD 70
#define MAXAREATHRESHOLD 2000

class CArucoMarkerNavigator
{
public:
	CArucoMarkerNavigator(std::string nodename) :
	m_strNode_Name(nodename)
	{
		m_SetSpeedClient = m_nNodeHandle.serviceClient <wheels::cmd_set_car_direction_speed>("set_direction_speed");
		m_CmdVelSubscriber = m_nNodeHandle.subscribe<geometry_msgs::Twist>("wheels_cmd_vel", 1000, 
											boost::bind(&CArucoMarkerNavigator::wheels_statusCallback, this, _1));
		//ROS_INFO("subscribe done");
		m_bCarStopped = true;
	}
	void wheels_statusCallback(const geometry_msgs::Twist::ConstPtr& msg)
	{
		m_LastMsgTime = time(NULL);
		//ROS_INFO("arucomarker_navigator:[angular:%f, linear:%f]", msg->angular.z, msg->linear.x);
		OffsetNavigator(m_SetSpeedClient, msg->angular.z, msg->linear.x);
		m_bCarStopped = false;
	}
	
	void Checklongpause(void)
	{
		time_t now = time(NULL);
		
		if (difftime(now, m_LastMsgTime) > 5 && m_bCarStopped == false)
		{
			wheels::cmd_set_car_direction_speed srv2;
			
			srv2.request.nNewSpeed = 0;	// zero speed;		
			srv2.request.nNewDirection = 0;	// stop
			
			if (m_SetSpeedClient.call(srv2))
			{
				//ROS_INFO("Set New Car direction=%d speed=%d", srv2.request.nNewDirection, srv2.request.nNewSpeed);
				//ROS_INFO("Last car status: RetCode=%d: last_dir=%d, last_speed=%d", srv2.response.nRetCode, srv2.response.nLastDirection, srv2.response.nLastSpeed);
				//nRet = 1;
				ROS_INFO("Cannot receive cmd_vel, stop the car");
				m_bCarStopped = true;
			}
			else
			{
				ROS_ERROR("Failed to call service set_direction_speed");
			}			
		}

	}
	int32_t OffsetNavigator(ros::ServiceClient &client, float fAngular, float fLinear)
	{
		int32_t nRet = 0;
		
		wheels::cmd_set_car_direction_speed srv2;
		
		float fXOffset = fabs(fAngular);
		
		if (fAngular > 0)
			srv2.request.nNewDirection = 3;	// turn right
		else
			srv2.request.nNewDirection = 5;	// turn left
		
		srv2.request.nNewSpeed = 60;	// max speed;
			
		if (fXOffset > 0.2 && fXOffset < 0.4)	// If the offset is more than 30% on either side from the center of the image
		{
			srv2.request.nNewSpeed = 60;	// max speed;
		}
		else if(fXOffset > 0.4 && fXOffset < 0.6)	// If the offset is more than 50% on either side from the center of the image
		{
			srv2.request.nNewSpeed = 65;	// max speed;
		}
		else if(fXOffset > 0.6 && fXOffset < 0.8)	// If the offset is more than 70% on either side from the center of the image
		{
			srv2.request.nNewSpeed = 70;	// max speed;	
		}
		else if(fXOffset > 0.8)	// If the offset is more than 90% on either side from the center of the image
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
protected:
	ros::NodeHandle m_nNodeHandle;
	std::string m_strNode_Name;
	ros::ServiceClient m_SetSpeedClient;
	ros::Subscriber m_CmdVelSubscriber;
	time_t m_LastMsgTime;
	bool m_bCarStopped;
};
};
#endif
