using namespace std; 
#include "wheelstatusconsole.h"
#include "myutil.h"
int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheel status console");

	yisys_roswheels::CWheelStatusConsole console;
	ros::spin();

	myclearscreen();
	return 1;
}
