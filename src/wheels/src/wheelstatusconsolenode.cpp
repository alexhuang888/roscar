using namespace std; 
#include "wheelstatusconsole.h"
#include "myutil.h"
int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheel status console");

	yisys_roswheels::CWheelStatusConsole console;
	ros::spin();
	/*
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	*/
	myclearscreen();
	return 1;
}
