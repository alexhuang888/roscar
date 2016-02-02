#ifndef __CWHEELDRIVER_H__
#define __CWHEELDRIVER_H__
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "wheels/wheels_status.h"
#include "wheels/cmd_get_one_wheel_status.h"
#include "wheels/cmd_set_car_direction_speed.h"
#include <geometry_msgs/Twist.h>
#include "wheels/cmd_send_manual_instruction.h"
#pragma once
namespace yisys_roswheels
{
#define WHEELANGULAR_TOLERANCE (0.4)
#define WHEELPIDMAXSPEED (10)
#define RIGHTWHEELADJUSTRATIO 1.2
#define LONGCMDVELSPAUSE 2 // seconds)
// it accept several msg and decide how the wheels works.
// cmd_vel: wheels direction and velocity
// ???: sign
// ???: manual override
class CWheelDriver
{
public:
	CWheelDriver(std::string nodename);
	virtual ~CWheelDriver() {};
	void PrintHelpInfo(void);
	void wheels_CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
	bool cbSendManualInstruction(wheels::cmd_send_manual_instructionRequest &req,
								wheels::cmd_send_manual_instructionResponse &res);
	void Checklongpause(void);

	// fAngular: angular direction and speed
	//				fAngular value: positive: right turn, negative: left turn. value is from 0-90 or 0 to -90
	//											when fabs(fAngular) is over tolerance, then it is decide to turn
	// fLinear: linear direction and speed (Positive: forward, negative: backward)
	//			fLinear value: normalized (0..1), mapping from 0 to 100 for wheel speed

	int32_t CmdVelToWheelController(float fAngular, float fLinear);

	int32_t KeyCodeToWheelController(unsigned char cKeyCode);

	int32_t GetWheelStatus(uint32_t nWheelID, wheels::cmd_get_one_wheel_status &Status);
protected:
	int32_t _SetSpeedDirection(int32_t nSpeed, int32_t nDirection);
	int32_t _SetInternalSpeed(int32_t nSpeed);
	void _ResetPIDParams(void);
protected:
	ros::NodeHandle m_nNodeHandle;
	std::string m_strNode_Name;
	ros::ServiceClient m_GetWheelStatusClient;
	ros::ServiceClient m_GetNavigatorEngineStatusClient;
	ros::ServiceClient m_SetSpeedClient;
	ros::ServiceClient m_SetTwoWheelsSpeedClient;
	ros::ServiceClient m_SetNavigatorEngine;
	ros::ServiceClient m_AskNavigatorEngineSaveImage;
	ros::ServiceServer m_SendManualInstructionService;
	ros::Subscriber m_CmdVelSubscriber;
	time_t m_LastMsgTime;
	bool m_bCarStopped;
	int32_t m_nCurrentUserSpeed;
	int32_t m_nCurrentUserDirection;
	bool m_bManualStop;	// true: to prevent the car to run dis-regarding any input cmd_vels
	bool m_bDisplayDebugImage;

	/*
	 * PID controller: speed = Kp * shift error + Ki * (accumulated error) + Kd * (d(Shift_Error) / dt)
	 * Kp is supposed to be FULLSPEED / 0.5 = 200;
	 */
    bool m_bPIDReset;
	float m_fThisShiftError;
	float m_fLastShiftError;
	float m_fAccumulatedShiftError;
	float m_fKp;
	float m_fKi;
	float m_fKd;
	float m_fRightWheelAdjustRatio;
	int32_t m_nFileSaveCounter;
};
};
#endif
