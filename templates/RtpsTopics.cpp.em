@###############################################
@#
@# EmPy template for generating RtpsTopics.cpp file
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - spec (msggen.MsgSpec) Parsed specification of the .msg file
@###############################################
@{
import genmsg.msgs
import os
from generate_uorb_topic_files import MsgScope # this is in Tools/

send_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND]
recv_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.RECEIVE]
}@


#include "RtpsTopics.h"

bool RtpsTopics::init(std::condition_variable *t_send_queue_cv, std::mutex *t_send_queue_mutex,
		      std::queue<uint8_t> *t_send_queue, const std::string &ns)
{
@[if recv_topics]@
	// Initialise subscribers
	std::cout << "\033[0;36m---   Subscribers   ---\033[0m" << std::endl;
@[for topic in recv_topics]@

	if (_@(topic)_sub.init(@(msgs[0].index(topic) + 1), t_send_queue_cv, t_send_queue_mutex, t_send_queue, ns)) {
		std::cout << "- @(topic) subscriber started" << std::endl;

	} else {
		std::cerr << "Failed starting @(topic) subscriber" << std::endl;
		return false;
	}

@[end for]@
	std::cout << "\033[0;36m-----------------------\033[0m" << std::endl << std::endl;
@[end if]@
@[if send_topics]@

	// Initialise publishers
	std::cout << "\033[0;36m----   Publishers  ----\033[0m" << std::endl;
@[for topic in send_topics]@

@[    if topic == 'Timesync' or topic == 'timesync']@
	if (_@(topic)_pub.init(ns)) {
		if (_@(topic)_fmu_in_pub.init(ns, std::string("fmu/timesync/in"))) {
			_timesync->start(&_@(topic)_fmu_in_pub);
			std::cout << "- @(topic) publishers started" << std::endl;
		}
@[    elif topic == 'TimesyncStatus' or topic == 'timesync_status']@
	if (_@(topic)_pub.init(ns, std::string("timesync_status"))) {
		_timesync->init_status_pub(&_@(topic)_pub);
		std::cout << "- @(topic) publisher started" << std::endl;
@[    elif topic == 'SensorDelta' or topic == 'sensor_delta']@
	if (_@(topic)_pub.init(ns)) {
		if(_transform != nullptr) {
			_transform->start();
		}
		std::cout << "- @(topic) publisher started" << std::endl;
@[    else]@
	if (_@(topic)_pub.init(ns)) {
		std::cout << "- @(topic) publisher started" << std::endl;
@[    end if]@

	} else {
		std::cerr << "ERROR starting @(topic) publisher" << std::endl;
		return false;
	}

@[end for]@
	std::cout << "\033[0;36m-----------------------\033[0m" << std::endl;
@[end if]@
	return true;
}

@[if send_topics]@
template <typename T>
void RtpsTopics::sync_timestamp_of_incoming_data(T &msg) {
	uint64_t timestamp = getMsgTimestamp(&msg);
	uint64_t timestamp_sample = getMsgTimestampSample(&msg);
	_timesync->subtractOffset(timestamp);
	setMsgTimestamp(&msg, timestamp);
	_timesync->subtractOffset(timestamp_sample);
	setMsgTimestampSample(&msg, timestamp_sample);
}

void RtpsTopics::publish(const uint8_t topic_ID, char data_buffer[], size_t len)
{
	switch (topic_ID) {
@[for topic in send_topics]@

	case @(msgs[0].index(topic) + 1): { // @(topic) publisher
		@(topic)_msg_t st;
		eprosima::fastcdr::FastBuffer cdrbuffer(data_buffer, len);
		eprosima::fastcdr::Cdr cdr_des(cdrbuffer);
		st.deserialize(cdr_des);
@[    if topic == 'Timesync' or topic == 'timesync']@
		_timesync->processTimesyncMsg(&st, &_@(topic)_pub);
@[    end if]@

		// apply timestamp offset
		sync_timestamp_of_incoming_data(st);

@[    if topic == 'SensorDelta' or topic == 'sensor_delta']@
		if(_transform != nullptr) {
			_transform->publish(&st);
		} else {
			_@(topic)_pub.publish(&st);
		}
@[    else]@
		_@(topic)_pub.publish(&st);
@[    end if]@
	}
	break;
@[end for]@

	default:
		printf("\033[1;33m[   micrortps_agent   ]\tUnexpected topic ID '%hhu' to publish. Please make sure the agent is capable of parsing the message associated to the topic ID '%hhu'\033[0m\n",
		       topic_ID, topic_ID);
		break;
	}
}
@[end if]@
@[if recv_topics]@
template <typename T>
void RtpsTopics::sync_timestamp_of_outgoing_data(T &msg) {
	uint64_t timestamp = getMsgTimestamp(&msg);
	uint64_t timestamp_sample = getMsgTimestampSample(&msg);
	_timesync->addOffset(timestamp);
	setMsgTimestamp(&msg, timestamp);
	_timesync->addOffset(timestamp_sample);
	setMsgTimestampSample(&msg, timestamp_sample);
}

bool RtpsTopics::getMsg(const uint8_t topic_ID, eprosima::fastcdr::Cdr &scdr)
{
	bool ret = false;

	switch (topic_ID) {
@[for topic in recv_topics]@

	case @(msgs[0].index(topic) + 1): // @(topic) subscriber
		if (_@(topic)_sub.hasMsg()) {
			@(topic)_msg_t msg = _@(topic)_sub.getMsg();

			// apply timestamp offset
			sync_timestamp_of_outgoing_data(msg);

			msg.serialize(scdr);
			ret = true;

			_@(topic)_sub.unlockMsg();
		}

		break;
@[end for]@

	default:
		printf("\033[1;33m[   micrortps_agent   ]\tUnexpected topic ID '%hhu' to getMsg. Please make sure the agent is capable of parsing the message associated to the topic ID '%hhu'\033[0m\n",
		       topic_ID, topic_ID);
		break;
	}

	return ret;
}
@[end if]@
