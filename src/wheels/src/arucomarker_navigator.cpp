using namespace std; 
#include "wheels/arucomarkernavigator.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "arucomarker_navigator");

	yisys_roswheels::CArucoMarkerNavigator navigator(ros::this_node::getName());
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		navigator.Checklongpause();
		loop_rate.sleep();
	}
	return 1;
}
