#ifndef __CWHEELNAVIGATOR_H__
#define __CWHEELNAVIGATOR_H__
#pragma once
#include "ros/ros.h"
#include "cnavigatorenginebase.h"
#include "wheels/cmd_get_navigator_engine_status.h"
#include "wheels/cmd_set_navigator_engine.h"
#include "wheels/cmd_ask_navigator_saveimage.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
// here, we would like to design a navigator fasade, which can include several
// navigator, then advertise "wheel_cmd_vel" to wheel driver.
// among all these navigator, they are:
// 1. Arcu tracker
// 2. Line detector
// 3. Road detector

#include "map"

namespace yisys_roswheels
{
typedef std::pair<uint32_t, CNavigatorEngineImplementationBase *> EngineImpPoolPairType;
typedef std::map<uint32_t, CNavigatorEngineImplementationBase *> EngineImpPoolType;
typedef std::map<uint32_t, CNavigatorEngineImplementationBase *>::iterator EngineImpPoolIteratorType;


class CWheelNavigator : public CNavigatorCallback
{
public:
	CWheelNavigator();
	virtual ~CWheelNavigator();

	int32_t SetTrackingEngine(uint32_t nEngineID);
	uint32_t GetTrackingEngineID(void);

	int32_t Init(void);
	int32_t StopAll(void) { return SetTrackingEngine(0); };

	bool cbSetEngine(wheels::cmd_set_navigator_engineRequest &req,
							wheels::cmd_set_navigator_engineResponse &res);

	bool cbGetEngineStatus(wheels::cmd_get_navigator_engine_statusRequest &req,
								wheels::cmd_get_navigator_engine_statusResponse &res);

	bool cbAskEngineSaveImage(wheels::cmd_ask_navigator_saveimageRequest &req,
							wheels::cmd_ask_navigator_saveimageResponse &res);

	void PublishEngineStatus(void);

public:
	virtual int32_t ProcessCmdVels(const geometry_msgs::Twist &velMsg);
	virtual int32_t PublishDebugImage(const sensor_msgs::ImagePtr);
protected:
	EngineImpPoolType m_EnginePool;
	uint32_t m_ActiveEngineID;
	CNavigatorEngineWithImageSource m_TrackingEngine;

	ros::NodeHandle m_nNodeHandle;
	ros::Publisher m_EngineStatusPublisher;
	ros::ServiceServer m_SetNavigatorEngineService;
	ros::ServiceServer m_AskNavigatorSaveImageService;
	ros::ServiceServer m_GetNavigatorEngineService;
	ros::Publisher m_WheelCmdVelPublisher;
	image_transport::Publisher m_ImageMsgPublisher;
	image_transport::ImageTransport m_ImageTransport;
};
};
#endif
