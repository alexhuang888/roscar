#include "ros/ros.h"
#include <sstream>
#include <termios.h>
#include "wheeldriver.h"
#include "myutil.h"
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
	ros::init(argc, argv, "wheel manual instruction");
	ros::NodeHandle nHandle;
	
	ros::ServiceClient sendinstClient = nHandle.serviceClient <wheels::cmd_send_manual_instruction>("send_manual_instruction");
	wheels::cmd_send_manual_instruction srv;
	
	uint32_t nInput = 0;

	while ((nInput = mygetch()) != 27)
	{
		srv.request.nManualInstruction = nInput;
		
		if (sendinstClient.call(srv) == false)
		{
			myprintf(_ERROR_LINENO, 1, "Fail to send manual instruction\n");
		}
	}
  return 0;
}
