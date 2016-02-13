#include "ros/ros.h"
#include "std_msgs/String.h"
#include "wheeldriver.h"
#include "wheelcontroller.h"
#include "wheelmotorengine.h"
#include "wheels/cmd_get_navigator_engine_status.h"
#include "wheels/cmd_set_navigator_engine.h"
#include "wheels/cmd_ask_navigator_saveimage.h"
#include "clinefollowernavigatorengine.h"
#include "clinefollowernavigatorengine2.h"
#include "clanedetectornavigatorengine.h"
#include "globalinc.h"
#include "myutil.h"

#define _THISFILE_LINENO _APPLINENO_1
#define _THISINFO_BLOCKLINENO _APPLINENO_5
namespace yisys_roswheels
{
CWheelDriver::CWheelDriver(std::string nodename) :
							m_strNode_Name(nodename)
{
	m_GetWheelStatusClient = m_nNodeHandle.serviceClient<wheels::cmd_get_one_wheel_status>("get_one_wheel_status");

	m_GetNavigatorEngineStatusClient = m_nNodeHandle.serviceClient<wheels::cmd_get_navigator_engine_status>("get_navigator_engine_status");

	m_SetSpeedClient = m_nNodeHandle.serviceClient <wheels::cmd_set_car_direction_speed>("set_direction_speed");

	m_SetTwoWheelsSpeedClient = m_nNodeHandle.serviceClient <wheels::cmd_set_car_two_wheels_direction_speed>("set_two_wheels_direction_speed");

	m_SetNavigatorEngine = m_nNodeHandle.serviceClient <wheels::cmd_set_navigator_engine>("set_navigator_engine");

    m_AskNavigatorEngineSaveImage = m_nNodeHandle.serviceClient <wheels::cmd_ask_navigator_saveimage>("ask_navigator_saveimage");

	m_CmdVelSubscriber = m_nNodeHandle.subscribe<geometry_msgs::Twist>("wheels_cmd_vel", 1000,
										boost::bind(&CWheelDriver::wheels_CmdVelCallback, this, _1));

	m_SendManualInstructionService = m_nNodeHandle.advertiseService("send_manual_instruction", &CWheelDriver::cbSendManualInstruction, this);

	m_bCarStopped = true;
	m_nCurrentUserSpeed = 0;
	m_nCurrentUserDirection = WCLR_STOP;
	m_bManualStop = false;
	m_bDisplayDebugImage = false;

    _ResetPIDParams();

	m_fKp = WHEELPIDMAXSPEED;
	m_fKi = 0.0001f;
	m_fKd = m_fKp / 2;
	m_fRightWheelAdjustRatio = RIGHTWHEELADJUSTRATIO;
	m_nFileSaveCounter = 0;
}

bool CWheelDriver::cbSendManualInstruction(wheels::cmd_send_manual_instructionRequest &req,
							wheels::cmd_send_manual_instructionResponse &res)
{
	res.nRetCode = KeyCodeToWheelController(req.nManualInstruction);

	return true;
}

void CWheelDriver::wheels_CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    Checklongpause();

	m_LastMsgTime = time(NULL);
	//printf("CWheelDriver: z=%f, x=%f\n", msg->angular.z, msg->linear.x);
	CmdVelToWheelController(msg->angular.z, msg->linear.x);
}
int32_t CWheelDriver::_SetInternalSpeed(int32_t nSpeed)
{
	int32_t nRet = 0;

	wheels::cmd_set_car_direction_speed srv2;

	if (m_bManualStop)
	{
		nRet = 1;
		goto err_out;
	}

	if (nSpeed > FULLSPEED)
		nSpeed = FULLSPEED;

	if (nSpeed < 0)
		nSpeed = 0;

	m_nCurrentUserSpeed = nSpeed;
	//m_nCurrentUserDirection = nDirection;
#if 0
	srv2.request.nNewSpeed = nSpeed;
	srv2.request.nNewDirection = nDirection;
	//printf("New Speed=%d, New Dir = %d\n", nSpeed, nDirection);
	if (m_SetSpeedClient.call(srv2))
	{
		m_nCurrentUserSpeed = nSpeed;
		m_nCurrentUserDirection = nDirection;

		if (nSpeed == 0 || m_nCurrentUserDirection == WCLR_STOP)
		{
			m_bCarStopped = true;
		}
		nRet = 1;
	}
	else
	{
		ROS_ERROR("Failed to call service set_direction_speed");
	}
#endif
err_out:
	return nRet;
}
int32_t CWheelDriver::_SetSpeedDirection(int32_t nSpeed, int32_t nDirection)
{
	int32_t nRet = 0;

	wheels::cmd_set_car_direction_speed srv2;

	if (m_bManualStop)
	{
		nRet = 1;
		goto err_out;
	}

	if (nSpeed > FULLSPEED)
		nSpeed = FULLSPEED;

	if (nSpeed < 0)
		nSpeed = 0;

	m_nCurrentUserSpeed = nSpeed;
	m_nCurrentUserDirection = nDirection;
#if 1
	srv2.request.nNewSpeed = nSpeed;
	srv2.request.nNewDirection = nDirection;
	//printf("New Speed=%d, New Dir = %d\n", nSpeed, nDirection);
	if (m_SetSpeedClient.call(srv2))
	{
		m_nCurrentUserSpeed = nSpeed;
		m_nCurrentUserDirection = nDirection;

		if (nSpeed == 0 || m_nCurrentUserDirection == WCLR_STOP)
		{
			m_bCarStopped = true;
		}
		nRet = 1;
	}
	else
	{
		myprintf(_ERRORLINENO, 1, "Failed to call service set_direction_speed");
	}
#endif
err_out:
	return nRet;
}
void CWheelDriver::Checklongpause(void)
{
	time_t now = time(NULL);

	if (difftime(now, m_LastMsgTime) > LONGCMDVELSPAUSE && m_bCarStopped == false)
	{
		//_SetSpeedDirection(0, WCLR_STOP);
		myprintf(_ERRORLINENO, 1, "Does not receive nagivator cmd_vels for over %d seconds. Stop the car.", LONGCMDVELSPAUSE);
		// we have to deal with something
		// 1. pause the car
		// 2. clear accumulated error
		_ResetPIDParams();
		m_nCurrentUserSpeed = 0;
		m_nCurrentUserDirection = 0;
	}
}
void CWheelDriver::_ResetPIDParams(void)
{
    m_fThisShiftError = 0;
    m_fLastShiftError = 0;
    m_fAccumulatedShiftError = 0;
    m_bPIDReset = true;
}
/*
PID controller: input is angle, and output is adjusted angle.
how should I map this adjusted angle back to differential wheel speed?

The idea is: we should plan our path first, then define a navigation plan. (turn x degree in t1 second, 
go straight in t2 seconds, turn y degree in t3 seconds.)

For each "action" in a navigation plan, we have to calcualte the speed of each wheels.
Then, feed the calculated speed of each wheel to its PID controllers.
*/
int32_t CWheelDriver::CmdVelToWheelController(float fAngular, float fLinear)
{
	int32_t nRet = 0;

	wheels::cmd_set_car_two_wheels_direction_speed srv2;

	float fErrorDiff;
	float nNewSpeed;
	float nNewRightSpeed, nNewLeftSpeed;
	int32_t nNewRightDirection, nNewLeftDirection;

	int32_t nDir = CMC_MOTORFORWARD;

	if (m_bManualStop || m_nCurrentUserSpeed == 0)
	{
		nRet = 1;
		goto err_out;
	}
	if (m_bPIDReset)
	{
        m_fLastShiftError = fAngular;
        m_bPIDReset = false;
	}
// fAngular: negative (target shift to left, turn left), or Positive (should turn right)
	m_fThisShiftError = (fAngular);

	m_fAccumulatedShiftError += m_fThisShiftError;

	fErrorDiff = (m_fThisShiftError - m_fLastShiftError);

	//m_fKp = m_nCurrentUserSpeed;
	nNewSpeed = m_fKp * m_fThisShiftError + m_fKi * m_fAccumulatedShiftError + m_fKd * fErrorDiff;

	m_fLastShiftError = m_fThisShiftError;


	if (fLinear < 0)
		nDir = CMC_MOTORBACKWARD;

	if (nNewSpeed < 0) // left turn
	{
		nNewRightSpeed = -nNewSpeed;
		nNewRightDirection = nDir;

		//nNewLeftSpeed = WHEELPIDMAXSPEED - nNewRightSpeed;
		nNewLeftSpeed = nNewSpeed;
		nNewLeftDirection = nDir;
	}
	else
	{
		nNewLeftSpeed = nNewSpeed;
		nNewLeftDirection = nDir;

		//nNewRightSpeed = WHEELPIDMAXSPEED - nNewLeftSpeed;
		nNewRightSpeed = -nNewSpeed;
		nNewRightDirection = nDir;
	}
	if (m_nCurrentUserSpeed == 0)	// users prefer to stop
	{
		nNewRightSpeed = 0;
		nNewLeftSpeed = 0;
	}
	else
	{
		nNewRightSpeed += m_nCurrentUserSpeed;
		nNewLeftSpeed += m_nCurrentUserSpeed;
	}

	nNewRightSpeed *= m_fRightWheelAdjustRatio;

	if (nNewRightSpeed > FULLSPEED)
		nNewRightSpeed = FULLSPEED;
	if (nNewRightSpeed < 0)
		nNewRightSpeed = 0;

	if (nNewLeftSpeed > FULLSPEED)
		nNewLeftSpeed = FULLSPEED;
	if (nNewLeftSpeed < 0)
		nNewLeftSpeed = 0;

	srv2.request.nNewLeftSpeed = nNewLeftSpeed;
	srv2.request.nNewLeftDirection = nNewLeftDirection;

	srv2.request.nNewRightSpeed = nNewRightSpeed;
	srv2.request.nNewRightDirection = nNewRightDirection;

	myprintf(_THISFILE_LINENO, 1, "Set wheel speed (%f, %f, %f, %f) Left(%f, %d), right(%f, %d)\n", fAngular, nNewSpeed, fErrorDiff, m_fAccumulatedShiftError, nNewLeftSpeed, nNewLeftDirection, nNewRightSpeed, nNewRightDirection);
	if (m_SetTwoWheelsSpeedClient.call(srv2))
	{
		if (srv2.response.nNewLeftSpeed == 0 && srv2.response.nNewRightSpeed == 0)
		{
			m_bCarStopped = true;
		}
		nRet = 1;
	}
	else
	{
		myprintf(_ERRORLINENO, 1, "Failed to call service set_two_wheels_direction_speed");
	}
err_out:


	return nRet;
}
int32_t CWheelDriver::KeyCodeToWheelController(unsigned char nInput)
{
	int32_t nRet = 0;

	int32_t nNewSpeed = m_nCurrentUserSpeed;
	int32_t nNewDirection = m_nCurrentUserDirection;
	//printf("input=%d\n", nInput);
	switch (nInput)
	{
        case 'c':
            {
                printf("\033[2J");
                printf("\n");
                break;
            }
		case 'y':	// display debug image
            {
                m_bDisplayDebugImage = !m_bDisplayDebugImage;
                m_nNodeHandle.setParam(WGP_DEBUG_SHOWIMAGE, m_bDisplayDebugImage);
            }
			break;
		case 'k':
			if (m_bManualStop)
			{
                _ResetPIDParams();
				m_bManualStop = false;
				myprintf(_THISINFO_BLOCKLINENO, 1, "UnBlock all incoming cmd_vels.");
			}
			break;
		case 'o':
			if (!m_bManualStop)
			{
                _ResetPIDParams();
				nNewSpeed = 0;
				nNewDirection = WCLR_STOP;
				nRet = _SetSpeedDirection(nNewSpeed, nNewDirection);
				m_bManualStop = true;
				myprintf(_THISINFO_BLOCKLINENO, 1, "Force to stop. Block all incoming cmd_vels.");
			}
			break;
		case 'p':
			if (!m_bManualStop)
			{
				nNewSpeed = 0;
				nNewDirection = WCLR_STOP;
				nRet = _SetSpeedDirection(nNewSpeed, nNewDirection);
				//ROS_INFO("Park the car.");
			}
			break;
		case '+':
			if (!m_bManualStop)
			{
				//printf("Before +, speed=%d, dir=%d\n", nNewSpeed, nNewDirection);
				nNewSpeed += 5;
				nRet = _SetInternalSpeed(nNewSpeed);
				//nRet = CmdVelToWheelController(0, 0);
				//ROS_INFO("Accelerate.");
			}
			break;
		case '-':
			if (!m_bManualStop)
			{
				nNewSpeed -= 5;
				nRet = _SetInternalSpeed(nNewSpeed);
				//nRet = CmdVelToWheelController(0, 0);
			}
			break;
		case 'u':
			if (!m_bManualStop)
			{
				nNewDirection = WCLR_FORWARD;
				//nRet = _SetSpeedDirection(nNewSpeed, nNewDirection);
				nRet = CmdVelToWheelController(0, 0);
			}
			break;
		case 'd':
			if (!m_bManualStop)
			{
				nNewDirection = WCLR_BACKWARD;
				//nRet = _SetSpeedDirection(nNewSpeed, nNewDirection);
				nRet = CmdVelToWheelController(0, -1);
			}
			break;
		case 'r':
			if (!m_bManualStop)
			{
				nNewDirection = WCLR_FORWARDRIGHTTURN;
				//nRet = _SetSpeedDirection(nNewSpeed, nNewDirection);
				nRet = CmdVelToWheelController(0.5, 0);
			}
			break;
		case 'l':
			if (!m_bManualStop)
			{
				nNewDirection = WCLR_FORWARDLEFTTURN;
				//nRet = _SetSpeedDirection(nNewSpeed, nNewDirection);
				nRet = CmdVelToWheelController(-0.5, 0);
			}
			break;
		case 'w':
			if (!m_bManualStop)
			{
				nNewDirection = WCLR_BACKWARDRIGHTTURN;
				//nRet = _SetSpeedDirection(nNewSpeed, nNewDirection);
				nRet = CmdVelToWheelController(0.5, -1);
			}
			break;
		case 'z':
			if (!m_bManualStop)
			{
				nNewDirection = WCLR_BACKWARDLEFTTURN;
				//nRet = _SetSpeedDirection(nNewSpeed, nNewDirection);
				nRet = CmdVelToWheelController(-0.5, -1);
			}
			break;
		case 'i':
		{
			wheels::cmd_get_one_wheel_status srv;
			wheels::cmd_get_navigator_engine_status engsrv;

			GetWheelStatus(CMC_LEFTWHEELID, srv);
			myprintf(_THISINFO_BLOCKLINENO, 1, "WheelID[%d] Ret=%d: dir=%d, speed=%d, health=%d", CMC_LEFTWHEELID, srv.response.nRetCode, srv.response.nWheelDirection, srv.response.nWheelSpeed, srv.response.nWheelHealthStatus);

			GetWheelStatus(CMC_RIGHTWHEELID, srv);
			myprintf(_THISINFO_BLOCKLINENO+1, 1, "WheelID[%d] Ret=%d: dir=%d, speed=%d, health=%d", CMC_RIGHTWHEELID, srv.response.nRetCode, srv.response.nWheelDirection, srv.response.nWheelSpeed, srv.response.nWheelHealthStatus);
			if (m_GetNavigatorEngineStatusClient.call(engsrv))
			{
				myprintf(_THISINFO_BLOCKLINENO+2, 1, "Active Navigator Engine, ID=%d [%s]", engsrv.response.nActiveEngineID, engsrv.response.strActiveEngineDescription.c_str());
			}
			else
			{
				myprintf(_THISINFO_BLOCKLINENO+2, 1, "Fail to query navigator engine status");
			}
			myprintf(_THISINFO_BLOCKLINENO+3, 1, "Current User Direction=%d, Current User Speed=%d", m_nCurrentUserDirection, m_nCurrentUserSpeed);
			if (m_bManualStop)
			{
				myprintf(_THISINFO_BLOCKLINENO+4, 1, "Manual stop is on");
			}
			else
			{
				myprintf(_THISINFO_BLOCKLINENO+4, 1, "Manual stop is off");
			}
			if (m_bDisplayDebugImage)
			{
				myprintf(_THISINFO_BLOCKLINENO+5, 1, "Debug image is on");
			}
			else
			{
				myprintf(_THISINFO_BLOCKLINENO+5, 1, "Debug image is off");
			}
			myprintf(_THISINFO_BLOCKLINENO+6, 1, "Right Wheel adjustment ratio=%f\n", m_fRightWheelAdjustRatio);
			break;
		}
		case '.':	// right wheel ratio + 0.01
			m_fRightWheelAdjustRatio += 0.01;
			nRet = CmdVelToWheelController(0, 0);
			break;
		case ',':
			m_fRightWheelAdjustRatio -= 0.01;
			nRet = CmdVelToWheelController(0, 0);
			break;
		case '0':	// disable all navigator engine
		{
			wheels::cmd_set_navigator_engine engset;
			if (!m_bManualStop)
			{
				nNewSpeed = 0;
				nNewDirection = WCLR_STOP;
				_SetSpeedDirection(nNewSpeed, nNewDirection);
				_ResetPIDParams();
			}
			//m_bManualStop = true;
			engset.request.nNewEngineID = 0;
			if (m_SetNavigatorEngine.call(engset))
			{
				/*
				ROS_INFO("Active Navigator Engine, ID=%d [%s], Old Navigator Engine ID=%d [%s]", engset.response.nActiveEngineID,
																								engset.response.strActiveEngineDescription.c_str(),
																								engset.response.nLastEngineID,
																								engset.response.strLastEngineDescription.c_str());
			*/
			}
			else
			{
				myprintf(_ERRORLINENO, 1, "Fail to set navigator engine");
			}

		}
			break;
		case '1':	// line follower
		{
			wheels::cmd_set_navigator_engine engset;

			engset.request.nNewEngineID = WEID_LINEFOLLOWERENGINE;
			if (m_SetNavigatorEngine.call(engset))
			{
                _ResetPIDParams();
				if (engset.response.nRetCode <= 0)
				{
					myprintf(_ERRORLINENO, 1, "fail to set Line follower engine");
				}
			}
			else
			{
				myprintf(_ERRORLINENO, 1, "Fail to set navigator engine");
			}
		}
			break;
		case '2':	// lane detector
		{
			wheels::cmd_set_navigator_engine engset;

			engset.request.nNewEngineID = WEID_LANEDETECTORENGINE;
			if (m_SetNavigatorEngine.call(engset))
			{
                _ResetPIDParams();
				if (engset.response.nRetCode <= 0)
				{
					myprintf(_ERRORLINENO, 1, "fail to set lane detector engine");
				}
			}
			else
			{
				myprintf(_ERRORLINENO, 1, "Fail to set navigator engine");
			}

		}
			break;
		case '3':	// line follower2
		{
			wheels::cmd_set_navigator_engine engset;

			engset.request.nNewEngineID = WEID_LINEFOLLOWERENGINE2;
			if (m_SetNavigatorEngine.call(engset))
			{
                _ResetPIDParams();
				if (engset.response.nRetCode <= 0)
				{
					myprintf(_ERRORLINENO, 1, "fail to set Line follower engine2");
				}
			}
			else
			{
				myprintf(_ERRORLINENO, 1, "Fail to set navigator engine2");
			}
		}
			break;
		case 'x':	// save image
		{
			wheels::cmd_ask_navigator_saveimage engsave;
            char szFile[200];

            sprintf(szFile, "/home/alex/RaspberryPiProject/imgout/img_%d.jpg", m_nFileSaveCounter);
			engsave.request.nModeFlags = 0;
			engsave.request.strNewImageFilename = std::string(szFile);
            m_nFileSaveCounter++;
            //printf("Ask to save file to %s\n", engsave.request.strNewImageFilename.c_str());
			if (m_AskNavigatorEngineSaveImage.call(engsave))
			{
				if (engsave.response.nRetCode <= 0)
				{
					myprintf(_ERRORLINENO, 1, "fail to save image");
				}
			}
			else
			{
				myprintf(_ERRORLINENO, 1, "Fail to set navigator engine");
			}
			break;
		}
		case 'h':
			//myprintf(19, 1, "Input instruction (0: disable all navigator engine, 1: line-follower, 2: lane-detector, o: manual stop, k: manual restart, u: forward, d: backward, l: left, r: right, w: right-backward, z: left-backward, p: stop, i: wheel status\n");
			PrintHelpInfo();
			break;
	}

	return nRet;
}
void CWheelDriver::PrintHelpInfo(void)
{
	myprintf(_THISINFO_BLOCKLINENO+7, 1, "Input instruction (0: disable all navigator engine, 1: line-follower, 2: lane-detector, o: manual stop, k: manual restart, u: forward, d: backward, l: left, r: right, w: right-backward, z: left-backward, p: stop, i: wheel status\n");
}
int32_t CWheelDriver::GetWheelStatus(uint32_t nWheelID, wheels::cmd_get_one_wheel_status &Status)
{
	int32_t nRet = 0;

	Status.request.nWheelID = nWheelID;

	if (m_GetWheelStatusClient.call(Status))
	{
		//ROS_INFO("WheelID[%d] Ret=%d: dir=%d, speed=%d, health=%d", CMC_LEFTWHEELID, srv.response.nRetCode, srv.response.nWheelDirection, srv.response.nWheelSpeed, srv.response.nWheelHealthStatus);
		nRet = 1;
	}
	else
	{
		myprintf(_ERRORLINENO, 1, "Failed to call service get_one_wheel_status");
	}
	return nRet;
}
}

