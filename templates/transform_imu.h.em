@###############################################
@#
@# EmPy template for generating <msg>transform_imu.cpp file
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - ros2_distro (str) ROS2 distro name
@#  - spec (msggen.MsgSpec) Parsed specification of the .msg file
@###############################################
@{
import genmsg.msgs
from packaging import version
import re

topic = alias if alias else spec.short_name
try:
    ros2_distro = ros2_distro.decode("utf-8")
except AttributeError:
    pass
}@


/*!
 * transform_imu.h
 * This header file contains the declaration of the publisher functions.
 *
 * This file was adapted from the fastrtpsgen tool.
 */


#ifndef _TRANSFORM_IMU__PUBLISHER_H_
#define _TRANSFORM_IMU__PUBLISHER_H_

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/publisher/PublisherListener.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sprain_msgs/msg/sensor_delta.hpp>

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

@[if version.parse(fastrtps_version) <= version.parse('1.7.2')]@
@[    if ros2_distro]@
using @(topic)_msg_t = @(package)::msg::dds_::@(topic)_;
using @(topic)_msg_datatype = @(package)::msg::dds_::@(topic)_PubSubType;
@[    else]@
using @(topic)_msg_t = @(topic)_;
using @(topic)_msg_datatype = @(topic)_PubSubType;
@[    end if]@
@[else]@
@[    if ros2_distro]@
using @(topic)_msg_t = @(package)::msg::@(topic);
using @(topic)_msg_datatype = @(package)::msg::@(topic)PubSubType;
@[    else]@
using @(topic)_msg_t = @(topic);
using @(topic)_msg_datatype = @(topic)PubSubType;
@[    end if]@
@[end if]@

class @(topic)_Publisher
{
public:
	@(topic)_Publisher();
	virtual ~@(topic)_Publisher();
	bool init(const std::string &ns, std::string topic_name = "");
	void run();
	void publish(@(topic)_msg_t *st);
private:
	Participant *mp_participant;
	Publisher *mp_publisher;

	class PubListener : public PublisherListener
	{
	public:
		PubListener() : n_matched(0) {};
		~PubListener() {};
		void onPublicationMatched(Publisher *pub, MatchingInfo &info);
		int n_matched;
	} m_listener;
	@(topic)_msg_datatype @(topic)DataType;
};

#endif // _@(topic)__PUBLISHER_H_
