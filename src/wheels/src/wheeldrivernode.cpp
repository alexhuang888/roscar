#include "ros/ros.h"
#include "myutil.h"
#include "wheeldriver.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheel driver");

	yisys_roswheels::CWheelDriver WheelDriver(ros::this_node::getName());
	uint32_t nInput = 0;

	myclearscreen();
	WheelDriver.PrintHelpInfo();
	ros::spin();
	myclearscreen();
	return 1;
}
