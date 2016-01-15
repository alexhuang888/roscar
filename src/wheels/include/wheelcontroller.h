#ifndef __WHEELS_STATUS_CALLBACK_HANDLER__
#define __WHEELS_STATUS_CALLBACK_HANDLER__
#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include "wheels/wheels_status.h"
#include "wheels/cmd_get_one_wheel_status.h"
#include "wheels/cmd_set_car_direction_speed.h"
#include "wheels/cmd_set_car_two_wheels_direction_speed.h"
#include "wheels/set_car_direction_speedAction.h"


namespace yisys_roswheels
{
#define WCLR_FORWARD 1
#define WCLR_BACKWARD 2
#define WCLR_FORWARDRIGHTTURN 3
#define WCLR_BACKWARDRIGHTTURN 4
#define WCLR_FORWARDLEFTTURN 5
#define WCLR_BACKWARDLEFTTURN 6
#define WCLR_STOP 100

#define WCLR_PARAM_MPINLA "MotorPinLeftA"
#define WCLR_PARAM_MPINLB "MotorPinLeftB"
#define WCLR_PARAM_MPINRA "MotorPinRightA"
#define WCLR_PARAM_MPINRB "MotorPinRightB"
class CTwoWheelsController;
class CWheelController
{
public:
	CWheelController(std::string name);
	virtual ~CWheelController();
	
	bool cbSetDirectionAndSpeed(wheels::cmd_set_car_direction_speedRequest &req,
							wheels::cmd_set_car_direction_speedResponse &res);

	
	bool cbSetTwoWheelsDirectionAndSpeed(wheels::cmd_set_car_two_wheels_direction_speedRequest &req,
							wheels::cmd_set_car_two_wheels_direction_speedResponse &res);
								
	bool cbGetOneWheelStatus(wheels::cmd_get_one_wheel_statusRequest &req,
								wheels::cmd_get_one_wheel_statusResponse &res);
								
	void goalCB();
	void preemptCB();
	void executeCB(const wheels::set_car_direction_speedGoalConstPtr &goal);
	void PublishWheelsStatus(void);
	
protected:
	int32_t cbSetDirectionAndSpeed(uint32_t, uint32_t);
protected:
	CTwoWheelsController *m_pGlobalCarController;
	ros::Publisher m_WheelStatusPublisher;
	ros::ServiceServer m_SetDirectionSpeedService;
	ros::ServiceServer m_SetTwoWheelsDirectionSpeedService;
	ros::ServiceServer m_GetOneWheelStatusService;
	ros::NodeHandle m_nNodeHandle;
	actionlib::SimpleActionServer<wheels::set_car_direction_speedAction> m_ActionServer;
	std::string m_strAction_Name;

};
};
#endif
