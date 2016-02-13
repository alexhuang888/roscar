#include "wheelnavigator.h"
#include "clinefollowernavigatorengine.h"
#include "clinefollowernavigatorengine2.h"
#include "clanedetectornavigatorengine.h"
#include "wheels/navigator_engine_status.h"
#include "wheels/wheels_status.h"

namespace yisys_roswheels
{
CWheelNavigator::CWheelNavigator() : m_ImageTransport(m_nNodeHandle)
{
	m_ActiveEngineID = 0;

	m_EngineStatusPublisher = m_nNodeHandle.advertise<wheels::navigator_engine_status>("navigator_engine_status", 1000);

	m_SetNavigatorEngineService = m_nNodeHandle.advertiseService("set_navigator_engine", &CWheelNavigator::cbSetEngine, this);

	m_AskNavigatorSaveImageService = m_nNodeHandle.advertiseService("ask_navigator_saveimage", &CWheelNavigator::cbAskEngineSaveImage, this);

	m_GetNavigatorEngineService = m_nNodeHandle.advertiseService("get_navigator_engine_status", &CWheelNavigator::cbGetEngineStatus, this);

	m_WheelCmdVelPublisher = m_nNodeHandle.advertise<geometry_msgs::Twist>("wheels_cmd_vel", 10);

	m_ImageMsgPublisher = m_ImageTransport.advertise("/wheels/debugimage", 1);

	m_TrackingEngine.SetNavigatorCallback(this);
}

CWheelNavigator::~CWheelNavigator()
{
	EngineImpPoolIteratorType it;

	for (it = m_EnginePool.begin(); it != m_EnginePool.end(); it++)
	{
		if (it->second != NULL)
			delete (it->second);
	}
	m_EnginePool.clear();
}

int32_t CWheelNavigator::Init(void)
{
	// line follower engine
	CLineFollowerNavigatorEngine *pLineFollowerEngine = NULL;
	CLineFollowerNavigatorEngine2 *pLineFollowerEngine2 = NULL;
	CLaneDetectorNavigatorEngine *pLaneDetectorEngine = NULL;

	int32_t nRet = 0;

	pLineFollowerEngine = new CLineFollowerNavigatorEngine;
	if (pLineFollowerEngine == NULL)
		goto err_out;


	m_EnginePool.insert(EngineImpPoolPairType(pLineFollowerEngine->GetEngineID(), pLineFollowerEngine));

	pLaneDetectorEngine = new CLaneDetectorNavigatorEngine;
	if (pLaneDetectorEngine == NULL)
		goto err_out;


	m_EnginePool.insert(EngineImpPoolPairType(pLaneDetectorEngine->GetEngineID(), pLaneDetectorEngine));

	pLineFollowerEngine2 = new CLineFollowerNavigatorEngine2;
	if (pLineFollowerEngine2 == NULL)
		goto err_out;


	m_EnginePool.insert(EngineImpPoolPairType(pLineFollowerEngine2->GetEngineID(), pLineFollowerEngine2));

	nRet = 1;

err_out:
	return nRet;
}

int32_t CWheelNavigator::SetTrackingEngine(uint32_t nEngineID)
{
	int32_t nRet = 0;
    CNavigatorEngineImplementationBase *pNavImpPtr = NULL;

	{
		m_TrackingEngine.Pause();
	}
	m_ActiveEngineID = 0;

	if (m_EnginePool.find(nEngineID) == m_EnginePool.end())
	{
		goto err_out;
	}
    pNavImpPtr = m_EnginePool.at(nEngineID);

	m_TrackingEngine.SetNavEngineImplementation(pNavImpPtr);
	m_ActiveEngineID = pNavImpPtr->GetEngineID();
	{
		m_TrackingEngine.Start();
	}

	PublishEngineStatus();
	nRet = 1;
err_out:
	return nRet;
}

uint32_t CWheelNavigator::GetTrackingEngineID(void)
{
	return m_ActiveEngineID;
}

bool CWheelNavigator::cbSetEngine(wheels::cmd_set_navigator_engineRequest &req,
									wheels::cmd_set_navigator_engineResponse &res)
{
	res.nLastEngineID = m_ActiveEngineID;

	res.strLastEngineDescription = m_TrackingEngine.GetEngineDescription();


	res.nRetCode = SetTrackingEngine(req.nNewEngineID);
	if (res.nRetCode > 0)
	{
		res.nActiveEngineID = req.nNewEngineID;

		res.strActiveEngineDescription = m_TrackingEngine.GetEngineDescription();
	}
	return true;
}


bool CWheelNavigator::cbAskEngineSaveImage(wheels::cmd_ask_navigator_saveimageRequest &req,
									wheels::cmd_ask_navigator_saveimageResponse &res)
{
    res.nRetCode = m_TrackingEngine.SaveImage(req.nModeFlags, req.strNewImageFilename);

	return true;
}

bool CWheelNavigator::cbGetEngineStatus(wheels::cmd_get_navigator_engine_statusRequest &req,
										wheels::cmd_get_navigator_engine_statusResponse &res)
{
	res.nActiveEngineID = m_ActiveEngineID;

	res.strActiveEngineDescription = m_TrackingEngine.GetEngineDescription();

	res.nRetCode = 1;

	return true;
}

void CWheelNavigator::PublishEngineStatus(void)
{
	wheels::navigator_engine_status status;

	status.nActiveEngineID = m_ActiveEngineID;

	status.strActiveEngineDescription = m_TrackingEngine.GetEngineDescription();

	m_EngineStatusPublisher.publish(status);
}

int32_t CWheelNavigator::ProcessCmdVels(const geometry_msgs::Twist &velMsg)
{
	m_WheelCmdVelPublisher.publish(velMsg);
	return 1;
}

int32_t CWheelNavigator::PublishDebugImage(const sensor_msgs::ImagePtr imgptr)
{
	m_ImageMsgPublisher.publish(imgptr);
	return 1;
}
}
