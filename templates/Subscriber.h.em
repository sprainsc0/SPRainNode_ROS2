@###############################################
@#
@# EmPy template for generating <msg>_uRTPS_UART.cpp file
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
 * @@file @(topic)_Subscriber.h
 * This header file contains the declaration of the subscriber functions.
 *
 * This file was adapted from the fastrtpsgen tool.
 */


#ifndef _@(topic)__SUBSCRIBER_H_
#define _@(topic)__SUBSCRIBER_H_

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/subscriber/SampleInfo.h>
@[if version.parse(fastrtps_version) <= version.parse('1.7.2')]@
#include "@(topic)_PubSubTypes.h"
@[else]@
#include "@(topic)PubSubTypes.h"
@[end if]@

#include <atomic>
#include <condition_variable>
#include <queue>

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

class @(topic)_Subscriber
{
public:
	@(topic)_Subscriber();
	virtual ~@(topic)_Subscriber();
	bool init(uint8_t topic_ID, std::condition_variable *t_send_queue_cv, std::mutex *t_send_queue_mutex,
		  std::queue<uint8_t> *t_send_queue, const std::string &ns, std::string topic_name = "");
	void run();
	bool hasMsg();
	@(topic)_msg_t getMsg();
	void unlockMsg();

private:
	Participant *mp_participant;
	Subscriber *mp_subscriber;

	class SubListener : public SubscriberListener
	{
	public:
		SubListener() : n_matched(0), n_msg(0), has_msg(false) {};
		~SubListener() {};
		void onSubscriptionMatched(Subscriber *sub, MatchingInfo &info);
		void onNewDataMessage(Subscriber *sub);
		SampleInfo_t m_info;
		int n_matched;
		int n_msg;
		@(topic)_msg_t msg;
		std::atomic_bool has_msg;
		uint8_t topic_ID;
		std::condition_variable *t_send_queue_cv;
		std::mutex *t_send_queue_mutex;
		std::queue<uint8_t> *t_send_queue;
		std::condition_variable has_msg_cv;
		std::mutex has_msg_mutex;

	} m_listener;
	@(topic)_msg_datatype @(topic)DataType;
};

#endif // _@(topic)__SUBSCRIBER_H_
