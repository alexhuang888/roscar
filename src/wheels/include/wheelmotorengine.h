/*
Raspberry pi wheels controller
*/
#include "stdint.h"
#include "map"
using namespace std;
namespace yisys_roswheels
{

#define CMC_MOTORIDLE 0
#define CMC_MOTORFORWARD 1
#define CMC_MOTORBACKWARD 2

#define RPI_GPIO_LOW LOW
#define RPI_GPIO_HIGH HIGH

#define FULLSPEED (100)
#define HALFSPEED ((FULLSPEED / 2))
#define LOWSPEED (0)

class CL298NMotorController
{
private:
	CL298NMotorController(){};
public:
	CL298NMotorController(uint32_t nForwardPinID, uint32_t nBackwardPinID)
	{
		m_nSpeed = 0;
		m_nDirection = 0;
		m_nHealthyStatus = 0;
		m_nForwardPinID = nForwardPinID;
		m_nBackwardPinID = nBackwardPinID;
		m_bInitialized = false;
	};
	virtual ~CL298NMotorController();
	
	uint32_t GetSpeed(void) { return m_nSpeed; };
	uint32_t GetDirection(void) { return m_nDirection; };
	uint32_t GetHealthyStatus(void)  { return m_nHealthyStatus; };
	
	int32_t SetSpeedAndDirection(uint32_t nNewSpeed, uint32_t nNewDirection);
	

	int32_t Reset(void);
	void GetGPIOPinID(uint32_t &nForwardPinID, uint32_t &nBackwardPinID);

	int32_t Initialize(void);
protected:
	uint32_t m_nSpeed;	// 0 - 100
	uint32_t m_nDirection;	// 0: Forward, 1: backward
	uint32_t m_nHealthyStatus;	// 0: unknown, 1: healthy
	
	uint32_t m_nForwardPinID;
	uint32_t m_nBackwardPinID;
	bool m_bInitialized;
};
#define CMC_LEFTWHEELID 1
#define CMC_RIGHTWHEELID 2
class CTwoWheelsController
{
private:
	CTwoWheelsController() {};
public:
	CTwoWheelsController(uint32_t nLeftForwardPin, uint32_t nLeftBackwardPin, uint32_t nRightForwardPin, uint32_t nRightBackwardPin);
	virtual ~CTwoWheelsController();
		
public:
	int32_t GetTotalMotors(void);
	int32_t Forward(uint32_t nSpeed);
	int32_t Backward(uint32_t nSpeed);
	int32_t TurnRight(uint32_t nSpeed, uint32_t nForwardBackward);
	int32_t TurnLeft(uint32_t nSpeed, uint32_t nForwardBackward);
	
	int32_t Stop(void);
	int32_t Reset(void);
	
	int32_t SetTwoWheelsSpeedDirection(uint32_t LeftSpeed, uint32_t LeftDirection, uint32_t RightSpeed, uint32_t RightDirection);
	void GetDirectionSpeed(uint32_t &nDirection, uint32_t &nSpeed)
	{
		nDirection = m_nDirection;
		nSpeed = m_nSpeed;
	}
	
	int32_t GetWheelStatus(uint32_t nWheelID, uint32_t &nDirection, uint32_t &nSpeed, uint32_t &nHealthStatus);
protected:
	CL298NMotorController &GetMotorControllers(uint32_t nWheelID);

protected:
	map<int32_t, CL298NMotorController*> m_Motors;
	uint32_t m_nDirection;	// 
	uint32_t m_nSpeed;	// 0 - 100
};
}
