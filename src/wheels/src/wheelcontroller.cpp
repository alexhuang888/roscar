/*
 * In this file, we do prepare to use service/client and action to do the job
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "wheelmotorengine.h"
#include "wheelcontroller.h"
#include "stdint.h"

namespace yisys_roswheels
{

CWheelController::CWheelController(std::string name) : 
    //m_ActionServer(m_nNodeHandle, name, boost::bind(&CWheelController::executeCB, this, _1), false),
    m_ActionServer(m_nNodeHandle, name, false),
    m_strAction_Name(name)
{
	// here, we would like to access GPIO Pin from parameter server, or from default value
	int nLA = 2, nLB = 3, nRA = 5, nRB = 6;
	
	m_nNodeHandle.param<int>(WCLR_PARAM_MPINLA, nLA, 2);
	m_nNodeHandle.param<int>(WCLR_PARAM_MPINLB, nLB, 3);
	m_nNodeHandle.param<int>(WCLR_PARAM_MPINRA, nRA, 5);
	m_nNodeHandle.param<int>(WCLR_PARAM_MPINRB, nRB, 6);
	
	m_pGlobalCarController = new CTwoWheelsController(nLA, nLB, nRA, nRB);

    //register the goal and feeback callbacks
    m_ActionServer.registerGoalCallback(boost::bind(&CWheelController::goalCB, this));
    m_ActionServer.registerPreemptCallback(boost::bind(&CWheelController::preemptCB, this));


    //m_ActionServer.start();
    
	m_WheelStatusPublisher = m_nNodeHandle.advertise<wheels::wheels_status>("wheels_status", 1000);

	m_SetDirectionSpeedService = m_nNodeHandle.advertiseService("set_direction_speed", &CWheelController::cbSetDirectionAndSpeed, this);

	m_SetTwoWheelsDirectionSpeedService = m_nNodeHandle.advertiseService("set_two_wheels_direction_speed", &CWheelController::cbSetTwoWheelsDirectionAndSpeed, this);
	
	m_GetOneWheelStatusService = m_nNodeHandle.advertiseService("get_one_wheel_status", &CWheelController::cbGetOneWheelStatus, this);    
}

CWheelController::~CWheelController()
{
	if (m_pGlobalCarController != NULL)
		delete m_pGlobalCarController;
	m_pGlobalCarController = NULL;
}

void CWheelController::PublishWheelsStatus(void)
{
	wheels::wheels_status status;

	if (m_pGlobalCarController!= NULL)
	{
		m_pGlobalCarController->GetWheelStatus(CMC_LEFTWHEELID, status.nLeftWheelDirection, status.nLeftWheelSpeed, status.nLeftWheelHealthStatus);
		m_pGlobalCarController->GetWheelStatus(CMC_RIGHTWHEELID, status.nRightWheelDirection, status.nRightWheelSpeed, status.nRightWheelHealthStatus);
		//ROS_INFO("Left[%d, %d, %d] Right[%d, %d, %d]", status.nLeftWheelDirection, status.nLeftWheelSpeed, status.nLeftWheelHealthStatus, status.nRightWheelDirection, status.nRightWheelSpeed, status.nRightWheelHealthStatus);
	}	
	/**
	* The publish() function is how you send messages. The parameter
	* is the message object. The type of this object must agree with the type
	* given as a template parameter to the advertise<>() call, as was done
	* in the constructor above.
	*/
	m_WheelStatusPublisher.publish(status);	
}

int32_t CWheelController::cbSetDirectionAndSpeed(uint32_t nNewDirection, uint32_t nNewSpeed)
{
	int32_t nRetCode = -1;
	
	if (m_pGlobalCarController != NULL)
	{
		//m_pGlobalCarController->GetDirectionSpeed(res.nLastDirection, res.nLastSpeed);
		
		switch (nNewDirection)
		{
			default:	// stop
			case WCLR_STOP:
				nRetCode = m_pGlobalCarController->Stop();
			break;
			case WCLR_FORWARD:	//forward
				nRetCode = m_pGlobalCarController->Forward(nNewSpeed);
			break;
			case WCLR_BACKWARD:	//backward
				nRetCode = m_pGlobalCarController->Backward(nNewSpeed);
			break;	
			case WCLR_FORWARDRIGHTTURN:	//forward, right turn
				nRetCode = m_pGlobalCarController->TurnRight(nNewSpeed, CMC_MOTORFORWARD);
			break;		
			case WCLR_BACKWARDRIGHTTURN:	//backward, right turn
				nRetCode = m_pGlobalCarController->TurnRight(nNewSpeed, CMC_MOTORBACKWARD);
			break;
			case WCLR_FORWARDLEFTTURN:	//forward, left turn
				nRetCode = m_pGlobalCarController->TurnLeft(nNewSpeed, CMC_MOTORFORWARD);
			break;		
			case WCLR_BACKWARDLEFTTURN:	//backward, left turn
				nRetCode = m_pGlobalCarController->TurnLeft(nNewSpeed, CMC_MOTORBACKWARD);
			break;			
		}
		//ROS_INFO("new request: direction=%d, speed=%d", nNewDirection, nNewSpeed);
		//PublishWheelsStatus();
		//ROS_INFO("sending back response: Code[%d], lastdirection=%d, lastspeed=%d", res.nRetCode, res.nLastDirection, res.nLastSpeed);
	}
	return nRetCode;		
}

bool CWheelController::cbSetDirectionAndSpeed(wheels::cmd_set_car_direction_speedRequest &req,
														wheels::cmd_set_car_direction_speedResponse &res)
{
	res.nRetCode = -1;
	if (m_pGlobalCarController != NULL)
	{
		m_pGlobalCarController->GetDirectionSpeed(res.nLastDirection, res.nLastSpeed);
		res.nRetCode = cbSetDirectionAndSpeed(req.nNewDirection, req.nNewSpeed);
		m_pGlobalCarController->GetDirectionSpeed(res.nNewDirection, res.nNewSpeed);
		//ROS_INFO("new request: direction=%d, speed=%d", req.nNewDirection, req.nNewSpeed);
		//ROS_INFO("sending back response: Code[%d], lastdirection=%d, lastspeed=%d", res.nRetCode, res.nLastDirection, res.nLastSpeed);
		return true;
	}
	return false;		
}
bool CWheelController::cbSetTwoWheelsDirectionAndSpeed(wheels::cmd_set_car_two_wheels_direction_speedRequest &req,
														wheels::cmd_set_car_two_wheels_direction_speedResponse &res)
{
	res.nRetCode = -1;
	if (m_pGlobalCarController != NULL)
	{
		uint32_t nHealthStatus;
		m_pGlobalCarController->GetWheelStatus(CMC_LEFTWHEELID, res.nLastLeftDirection, res.nLastLeftSpeed, nHealthStatus);
		m_pGlobalCarController->GetWheelStatus(CMC_RIGHTWHEELID, res.nLastRightDirection, res.nLastRightSpeed, nHealthStatus);

		res.nRetCode = m_pGlobalCarController->SetTwoWheelsSpeedDirection(req.nNewLeftSpeed, req.nNewLeftDirection, req.nNewRightSpeed, req.nNewRightDirection);
		
		m_pGlobalCarController->GetWheelStatus(CMC_LEFTWHEELID, res.nNewLeftDirection, res.nNewLeftSpeed, nHealthStatus);
		m_pGlobalCarController->GetWheelStatus(CMC_RIGHTWHEELID, res.nNewRightDirection, res.nNewRightSpeed, nHealthStatus);
		//ROS_INFO("new request: direction=%d, speed=%d", req.nNewDirection, req.nNewSpeed);
		//ROS_INFO("sending back response: Code[%d], lastdirection=%d, lastspeed=%d", res.nRetCode, res.nLastDirection, res.nLastSpeed);
		return true;
	}
	return false;		
}
bool CWheelController::cbGetOneWheelStatus(wheels::cmd_get_one_wheel_statusRequest &req,
														wheels::cmd_get_one_wheel_statusResponse &res)
{
	if (m_pGlobalCarController != NULL)
	{
		res.nRetCode = m_pGlobalCarController->GetWheelStatus(req.nWheelID, res.nWheelDirection, res.nWheelSpeed, res.nWheelHealthStatus);
		
		//ROS_INFO("request: wheelID=%d", req.nWheelID);
		//ROS_INFO("sending back response: RetCode=%d, WheelDirection=%d, wheel Speed=%d wheel healthStatus=%d", res.nRetCode, res.nWheelDirection, res.nWheelSpeed, res.nWheelHealthStatus);
		return true;
	}
	return false;
}	

void CWheelController::goalCB()
{
	// reset helper variables

	// accept the new goal
	//goal_ = m_ActionServer.acceptNewGoal()->samples;
}

void CWheelController::preemptCB()
{
	//ROS_INFO("%s: Preempted", m_strAction_Name.c_str());
	// set the action state to preempted
	//as_.setPreempted();
}

void CWheelController::executeCB(const wheels::set_car_direction_speedGoalConstPtr &goal)
{

}
};  

