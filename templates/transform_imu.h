/*!
 * transform_imu.h
 * This header file contains the declaration of the publisher functions.
 *
 * This file was adapted from the fastrtpsgen tool.
 */


#ifndef _TRANSFORM_IMU__PUBLISHER_H_
#define _TRANSFORM_IMU__PUBLISHER_H_

#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include "SensorDelta_Publisher.h"

using SensorDelta_msg_t = sprain_msgs::msg::SensorDelta;
using Imu_msg_t = sensor_msgs::msg::Imu;

class Transform_Imu
{
public:
	Transform_Imu();
	virtual ~Transform_Imu();

	void start();
	void stop();

	void publish(SensorDelta_msg_t *st);
private:
	std::shared_ptr<rclcpp::Node>  _transform_node;

	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_publisher;

	std::unique_ptr<std::thread>  _transform_node_thread;
};

#endif
