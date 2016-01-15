#ifndef __WHEELSTATUSCONSOLE_H__
#define __WHEELSTATUSCONSOLE_H__

#pragma once
#include "ros/ros.h"
#include "wheels/wheels_status.h"
#include "wheels/navigator_engine_status.h"
#include <geometry_msgs/Twist.h>
namespace yisys_roswheels
{
	// this class is to consolidate all status reports regarding roscar project
	// wheels status, camera status, tracking status, ...
	class CWheelStatusConsole
	{
		public:
			CWheelStatusConsole();
			virtual ~CWheelStatusConsole() {};
			void wheels_statusCallback(const wheels::wheels_statusConstPtr& msg);
			void navigator_engine_statusCallback(const wheels::navigator_engine_statusConstPtr& msg);
			void wheels_cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg);
		protected:
			ros::NodeHandle m_nNodeHandle;
			std::string m_strNode_Name;

			ros::Subscriber m_WheelStatusSubscriber;
			ros::Subscriber m_NavigatorEngineStatusSubscriber;
			ros::Subscriber m_WheelsCmdVelSubscriber;
			time_t m_LastMsgTime;
	};
};
#endif
