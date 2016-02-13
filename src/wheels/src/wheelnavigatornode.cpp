using namespace std;
#include "wheelnavigator.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheel navigator");

	yisys_roswheels::CWheelNavigator navigator;

	navigator.Init();

    ros::spin();
	return 1;
}
