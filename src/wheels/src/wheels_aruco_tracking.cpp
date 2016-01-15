#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
using namespace std; 
int main(int argc, char **argv)
{
	float fXOffset = 0, fYOffset = 0;
	int32_t nCenterX = 0, nCenterY = 0;
	uint32_t nTick = 0;

	ros::init(argc, argv, "moving_vel_aruco_tracker");

	ros::NodeHandle node;

	ros::Publisher wheels_vel = node.advertise<geometry_msgs::Twist>("wheels_cmd_vel", 10);
	
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	ros::Rate rate(10.0);
	while (node.ok())
	{
		geometry_msgs::TransformStamped transformStamped;
		try{
			ros::Time now = ros::Time::now();
			if (tfBuffer.canTransform("camera1", "board1", now, ros::Duration(3.0)))
			{
				transformStamped = tfBuffer.lookupTransform("camera1", "board1",
				now);
				
				geometry_msgs::Twist vel_msg;

				vel_msg.angular.z = atan2(transformStamped.transform.translation.x,
									transformStamped.transform.translation.z);
				vel_msg.linear.x = sqrt(pow(transformStamped.transform.translation.x, 2) +
									pow(transformStamped.transform.translation.z, 2));
				wheels_vel.publish(vel_msg);
				ROS_INFO("T(%f,%f,%f) R(%f,%f,%f,%f)", transformStamped.transform.translation.x, transformStamped.transform.translation.y, 
													transformStamped.transform.translation.z,
													transformStamped.transform.rotation.x,
													transformStamped.transform.rotation.y,
													transformStamped.transform.rotation.z,
													transformStamped.transform.rotation.w);
													
				ROS_INFO("vel_msg:angular:%f, linear:%f", vel_msg.angular.z, vel_msg.linear.x);				
			}
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}


		rate.sleep();
	}

	return 1;
}
