#include "stdlib.h"

#include "wheelmotorengine.h"

#include "wiringPi.h"
#include "softPwm.h"
namespace yisys_roswheels
{
// include gpio library

// implment all modules
CL298NMotorController::~CL298NMotorController()
{
	if (m_bInitialized)
	{
		pinMode(m_nForwardPinID, OUTPUT);
		pinMode(m_nBackwardPinID, OUTPUT);		
		softPwmWrite(m_nForwardPinID, RPI_GPIO_LOW);	
		softPwmWrite(m_nBackwardPinID, RPI_GPIO_LOW);		
	}
	m_bInitialized = false;
}

int32_t CL298NMotorController::Initialize(void)
{
	wiringPiSetupGpio();
	
	pinMode(m_nForwardPinID, OUTPUT);
	pinMode(m_nBackwardPinID, OUTPUT);
	pullUpDnControl(m_nForwardPinID, PUD_OFF);
	pullUpDnControl(m_nBackwardPinID, PUD_OFF);
	softPwmCreate(m_nForwardPinID, 0, FULLSPEED);
	softPwmCreate(m_nBackwardPinID, 0, FULLSPEED);	
	m_nDirection = CMC_MOTORIDLE;
	m_nSpeed = 0;		
	m_nHealthyStatus = 1;	// good condition
	m_bInitialized = true;
	return 1;
}


int32_t CL298NMotorController::SetSpeedAndDirection(uint32_t nNewSpeed, uint32_t nNewDirection)
{
	switch (nNewDirection)
	{
		case CMC_MOTORFORWARD:
			softPwmWrite(m_nForwardPinID, nNewSpeed);	
			softPwmWrite(m_nBackwardPinID, RPI_GPIO_LOW);				
		break;
		case CMC_MOTORBACKWARD:
			softPwmWrite(m_nBackwardPinID, nNewSpeed);	
			softPwmWrite(m_nForwardPinID, RPI_GPIO_LOW);			
		break;
	}
	m_nDirection = nNewSpeed == 0 ? CMC_MOTORIDLE : nNewDirection;
	m_nSpeed = nNewSpeed;
	return 1;
}
	
int32_t CL298NMotorController::Reset(void)
{
	pinMode(m_nForwardPinID, OUTPUT);
	pinMode(m_nBackwardPinID, OUTPUT);
	pullUpDnControl(m_nForwardPinID, PUD_OFF);
	pullUpDnControl(m_nBackwardPinID, PUD_OFF);
	softPwmWrite(m_nForwardPinID, RPI_GPIO_LOW);	
	softPwmWrite(m_nBackwardPinID, RPI_GPIO_LOW);	
	
	m_nDirection = CMC_MOTORIDLE;
	m_nSpeed = 0;			
	return 1;
}

void CL298NMotorController::GetGPIOPinID(uint32_t &nForwardPinID, uint32_t &nBackwardPinID)
{
	nForwardPinID = m_nForwardPinID;
	nBackwardPinID = m_nBackwardPinID;
}

int32_t CTwoWheelsController::GetWheelStatus(uint32_t nWheelID, uint32_t &nDirection, uint32_t &nSpeed, uint32_t &nHealthStatus)
{
	if (m_Motors[nWheelID] != NULL)
	{
		nDirection = m_Motors[nWheelID]->GetDirection();
		nSpeed = m_Motors[nWheelID]->GetSpeed();
		nHealthStatus = m_Motors[nWheelID]->GetHealthyStatus();
		
		return 1;
	}
	return 0;
}
/// car wheels controller
CTwoWheelsController::CTwoWheelsController(uint32_t nLeftForwardPin, uint32_t nLeftBackwardPin, uint32_t nRightForwardPin, uint32_t nRightBackwardPin)
{
	// create two new motors
	CL298NMotorController *pMotor = NULL;
	
	pMotor = new CL298NMotorController(nLeftForwardPin, nLeftBackwardPin);
	
	if (pMotor != NULL)
	{
		m_Motors.insert(pair<uint32_t, CL298NMotorController*>(CMC_LEFTWHEELID, pMotor));
		pMotor->Initialize();
	}
	pMotor = new CL298NMotorController(nRightForwardPin, nRightBackwardPin);
	
	if (pMotor != NULL)
	{
		m_Motors.insert(pair<uint32_t, CL298NMotorController*>(CMC_RIGHTWHEELID, pMotor));
		pMotor->Initialize();
	}	
	
	m_nDirection = 0;
	m_nSpeed = 0;
}

CTwoWheelsController::~CTwoWheelsController()
{
	map<int32_t, CL298NMotorController*>::iterator it;
	
	for (it = m_Motors.begin(); it != m_Motors.end(); it++)
	{
		delete (it->second);
	}
	m_Motors.clear();
}
		
int32_t CTwoWheelsController::GetTotalMotors(void)
{
	return m_Motors.size();
}
int32_t CTwoWheelsController::Forward(uint32_t nSpeed)
{
	m_Motors[CMC_LEFTWHEELID]->SetSpeedAndDirection(nSpeed, CMC_MOTORFORWARD);
	m_Motors[CMC_RIGHTWHEELID]->SetSpeedAndDirection(nSpeed, CMC_MOTORFORWARD);
	
	m_nDirection = 1;
	m_nSpeed = nSpeed;
	return 1;
}
int32_t CTwoWheelsController::Backward(uint32_t nSpeed)
{
	m_Motors[CMC_LEFTWHEELID]->SetSpeedAndDirection(nSpeed, CMC_MOTORBACKWARD);
	m_Motors[CMC_RIGHTWHEELID]->SetSpeedAndDirection(nSpeed, CMC_MOTORBACKWARD);

	m_nDirection = 2;
	m_nSpeed = nSpeed;
	return 1;	
}
int32_t CTwoWheelsController::TurnRight(uint32_t nSpeed, uint32_t nForwardBackward)
{
	m_Motors[CMC_LEFTWHEELID]->SetSpeedAndDirection(nSpeed, nForwardBackward);
	m_Motors[CMC_RIGHTWHEELID]->SetSpeedAndDirection(0, CMC_MOTORFORWARD);
	m_nDirection = nForwardBackward == CMC_MOTORFORWARD ? 3 : 4;
	m_nSpeed = nSpeed;	
	return 1;		
}
int32_t CTwoWheelsController::TurnLeft(uint32_t nSpeed, uint32_t nForwardBackward)
{
	m_Motors[CMC_LEFTWHEELID]->SetSpeedAndDirection(0, CMC_MOTORFORWARD);
	m_Motors[CMC_RIGHTWHEELID]->SetSpeedAndDirection(nSpeed, nForwardBackward);
	
	m_nDirection = nForwardBackward == CMC_MOTORFORWARD ? 5 : 6;
	m_nSpeed = nSpeed;	
	return 1;		
}
	
int32_t CTwoWheelsController::Stop(void)
{
	m_Motors[CMC_LEFTWHEELID]->SetSpeedAndDirection(0, CMC_MOTORFORWARD);
	m_Motors[CMC_RIGHTWHEELID]->SetSpeedAndDirection(0, CMC_MOTORFORWARD);
	
	//m_nDirection = nForwardBackward == CMC_MOTORFORWARD ? 3 : 4;
	m_nSpeed = 0;	
	return 1;		
}
int32_t CTwoWheelsController::Reset(void)
{
	m_Motors[CMC_LEFTWHEELID]->Reset();
	m_Motors[CMC_RIGHTWHEELID]->Reset();
	//m_nDirection = nForwardBackward == CMC_MOTORFORWARD ? 3 : 4;
	m_nDirection = 0;
	m_nSpeed = 0;	
	return 1;		
}
	
CL298NMotorController &CTwoWheelsController::GetMotorControllers(uint32_t nWheelID)
{
	return *(m_Motors[nWheelID]);
}

int32_t CTwoWheelsController::SetTwoWheelsSpeedDirection(uint32_t LeftSpeed, uint32_t LeftDirection, uint32_t RightSpeed, uint32_t RightDirection)
{
	m_Motors[CMC_LEFTWHEELID]->SetSpeedAndDirection(LeftSpeed, LeftDirection);
	m_Motors[CMC_RIGHTWHEELID]->SetSpeedAndDirection(RightSpeed, RightDirection);
		
	return 1;		
}
}
