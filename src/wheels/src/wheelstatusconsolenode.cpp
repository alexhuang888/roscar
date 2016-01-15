using namespace std; 
#include "wheelstatusconsole.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheel status console");

	yisys_roswheels::CWheelStatusConsole console;
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 1;
}
