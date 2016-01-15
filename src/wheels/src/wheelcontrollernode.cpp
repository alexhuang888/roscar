using namespace std; 
#include "wheelcontroller.h"

#include <sstream>
#include <termios.h>
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int mygetch (void) 
{
  int ch;
  struct termios oldt, newt;

  tcgetattr ( STDIN_FILENO, &oldt );
  newt = oldt;
  newt.c_lflag &= ~( ICANON | ECHO );
  tcsetattr ( STDIN_FILENO, TCSANOW, &newt );
  ch = getchar();
  tcsetattr ( STDIN_FILENO, TCSANOW, &oldt );

  return ch;
} 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheel_controller");

	yisys_roswheels::CWheelController cbHandler(ros::this_node::getName());
  
	ros::spin();

	return 0;
}
