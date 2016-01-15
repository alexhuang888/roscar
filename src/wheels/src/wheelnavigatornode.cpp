using namespace std;
#include "wheelnavigator.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheel navigator");

	yisys_roswheels::CWheelNavigator navigator;

	navigator.Init();
/*
	ros::Rate loop_rate(60);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
*/
    ros::spin();
	return 1;
}
