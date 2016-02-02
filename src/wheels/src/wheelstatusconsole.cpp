
#include "wheelstatusconsole.h"
#include "myutil.h"
#define _THISFILELINNO _INFOLINENO
namespace yisys_roswheels
{
	CWheelStatusConsole::CWheelStatusConsole()
	{
		m_WheelStatusSubscriber = m_nNodeHandle.subscribe<wheels::wheels_status>("wheels_status", 1000,
											boost::bind(&CWheelStatusConsole::wheels_statusCallback, this, _1));

		m_NavigatorEngineStatusSubscriber = m_nNodeHandle.subscribe<wheels::navigator_engine_status>("navigator_engine_status", 1000,
											boost::bind(&CWheelStatusConsole::navigator_engine_statusCallback, this, _1));

		m_WheelsCmdVelSubscriber = m_nNodeHandle.subscribe<geometry_msgs::Twist>("wheels_cmd_vel", 1000,
											boost::bind(&CWheelStatusConsole::wheels_cmd_velCallback, this, _1));
	}
	void CWheelStatusConsole::wheels_statusCallback(const wheels::wheels_statusConstPtr& msg)
	{
		try
		{
			m_LastMsgTime = time(NULL);
		} catch (...)
		{
		}
		myprintf(_THISFILELINNO, 1, "Broadcasted Wheel_Status (Dir, Speed, Health): Left[%d, %d, %d] Right[%d, %d, %d]", msg->nLeftWheelDirection, msg->nLeftWheelSpeed, msg->nLeftWheelHealthStatus, msg->nRightWheelDirection, msg->nRightWheelSpeed, msg->nRightWheelHealthStatus);
	}
	void CWheelStatusConsole::navigator_engine_statusCallback(const wheels::navigator_engine_statusConstPtr& msg)
	{
		try
		{
			m_LastMsgTime = time(NULL);
		} catch (...)
		{
		}
		myprintf(_THISFILELINNO+1, 1, "Broadcasted Navigator_Engine_Status: Active Engine ID= %d [%s]", msg->nActiveEngineID, msg->strActiveEngineDescription.c_str());
	}
	void CWheelStatusConsole::wheels_cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg)
	{
		try
		{
			m_LastMsgTime = time(NULL);
		} catch (...)
		{
		}
		myprintf(_THISFILELINNO + 2, 1, "wheels_cmd_vels: z=%3.3f, x=%3.3f", msg->angular.z, msg->linear.x);

	}
}

