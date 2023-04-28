@###############################################
@#
@# EmPy template for generating RtpsTopics.h file
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - fastrtps_version (List[str]) FastRTPS version installed on the system
@#  - package (List[str]) messages package name.
@#  - ros2_distro (List[str]) ROS2 distro name
@#  - spec (msggen.MsgSpec) Parsed specification of the .msg file
@###############################################
@{
import genmsg.msgs
import os
from packaging import version
from generate_uorb_topic_files import MsgScope # this is in Tools/

send_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND]
recv_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.RECEIVE]
package = package[0]
fastrtps_version = fastrtps_version[0]
try:
    ros2_distro = ros2_distro[0].decode("utf-8")
except AttributeError:
    ros2_distro = ros2_distro[0]
}@


#include <fastcdr/Cdr.h>
#include <condition_variable>
#include <queue>
#include <type_traits>

#include "microRTPS_timesync.h"
#include "transform_imu.h"

@[for topic in send_topics]@
#include "@(topic)_Publisher.h"
@[end for]@
@[for topic in recv_topics]@
#include "@(topic)_Subscriber.h"
@[end for]@


@[for topic in (recv_topics + send_topics)]@
@[    if version.parse(fastrtps_version) <= version.parse('1.7.2')]@
@[        if ros2_distro]@
using @(topic)_msg_t = @(package)::msg::dds_::@(topic)_;
@[        else]@
using @(topic)_msg_t = @(topic)_;
@[        end if]@
@[    else]@
@[        if ros2_distro]@
using @(topic)_msg_t = @(package)::msg::@(topic);
@[        else]@
using @(topic)_msg_t = @(topic);
@[        end if]@
@[    end if]@
@[end for]@

class RtpsTopics
{
public:
	bool init(std::condition_variable *t_send_queue_cv, std::mutex *t_send_queue_mutex, std::queue<uint8_t> *t_send_queue,
		  const std::string &ns);
	void set_timesync(const std::shared_ptr<TimeSync> &timesync) { _timesync = timesync; };
	void set_transform(const std::shared_ptr<Transform_Imu> &transform) { _transform = transform; };
@[if send_topics]@
	template <typename T>
	void sync_timestamp_of_incoming_data(T &msg);
	void publish(const uint8_t topic_ID, char data_buffer[], size_t len);
@[end if]@
@[if recv_topics]@
	template <typename T>
	void sync_timestamp_of_outgoing_data(T &msg);
	bool getMsg(const uint8_t topic_ID, eprosima::fastcdr::Cdr &scdr);
@[end if]@

private:
@[if send_topics]@
	/** Publishers **/
@[for topic in send_topics]@
@[    if topic == 'Timesync' or topic == 'timesync']@
	@(topic)_Publisher _@(topic)_pub;
	@(topic)_Publisher _@(topic)_fmu_in_pub;
@[    else]@
	@(topic)_Publisher _@(topic)_pub;
@[    end if]@
@[end for]@
@[end if]@

@[if recv_topics]@
	/** Subscribers **/
@[for topic in recv_topics]@
	@(topic)_Subscriber _@(topic)_sub;
@[end for]@
@[end if]@

	// SFINAE
	template<typename T> struct hasTimestampSample{
	private:
		template<typename U,
            typename = decltype(std::declval<U>().timestamp_sample(int64_t()))>
		static std::true_type detect(int);
		template<typename U>
		static std::false_type detect(...);
	public:
		static constexpr bool value = decltype(detect<T>(0))::value;
  };

	template<typename T>
	inline typename std::enable_if < !hasTimestampSample<T>::value, uint64_t >::type
	getMsgTimestampSample_impl(const T *) { return 0; }

	/** Msg metada Getters **/
@[if version.parse(fastrtps_version) <= version.parse('1.7.2') or not ros2_distro]@
	template <class T>
	inline uint64_t getMsgTimestamp(const T *msg) { return msg->timestamp_(); }

	template<typename T>
	inline typename std::enable_if<hasTimestampSample<T>::value, uint64_t>::type
	getMsgTimestampSample_impl(const T *msg) { return msg->timestamp_sample_(); }

	template <class T>
	inline uint8_t getMsgSysID(const T *msg) { return msg->sys_id_(); }

	template <class T>
	inline uint8_t getMsgSeq(const T *msg) { return msg->seq_(); }
@[elif ros2_distro]@
	template <class T>
	inline uint64_t getMsgTimestamp(const T *msg) { return msg->timestamp(); }

	template<typename T>
	inline typename std::enable_if<hasTimestampSample<T>::value, uint64_t>::type
	getMsgTimestampSample_impl(const T *msg) { return msg->timestamp_sample(); }

	template <class T>
	inline uint8_t getMsgSysID(const T *msg) { return msg->sys_id(); }

	template <class T>
	inline uint8_t getMsgSeq(const T *msg) { return msg->seq(); }
@[end if]@

	template <class T>
	inline uint64_t getMsgTimestampSample(const T *msg) { return getMsgTimestampSample_impl(msg); }

	template<typename T>
	inline typename std::enable_if <!hasTimestampSample<T>::value, void>::type
	setMsgTimestampSample_impl(T *, const uint64_t &) {}

	/** Msg metadata Setters **/
@[if version.parse(fastrtps_version) <= version.parse('1.7.2') or not ros2_distro]@
	template <class T>
	inline void setMsgTimestamp(T *msg, const uint64_t &timestamp) { msg->timestamp_() = timestamp; }

	template <class T>
	inline typename std::enable_if<hasTimestampSample<T>::value, void>::type
	setMsgTimestampSample_impl(T *msg, const uint64_t &timestamp_sample) { msg->timestamp_sample_() = timestamp_sample; }

	template <class T>
	inline void setMsgSysID(T *msg, const uint8_t &sys_id) { msg->sys_id_() = sys_id; }

	template <class T>
	inline void setMsgSeq(T *msg, const uint8_t &seq) { msg->seq_() = seq; }
@[elif ros2_distro]@
	template <class T>
	inline void setMsgTimestamp(T *msg, const uint64_t &timestamp) { msg->timestamp() = timestamp; }

	template <class T>
	inline typename std::enable_if<hasTimestampSample<T>::value, void>::type
	setMsgTimestampSample_impl(T *msg, const uint64_t &timestamp_sample) { msg->timestamp_sample() = timestamp_sample; }

	template <class T>
	inline void setMsgSysID(T *msg, const uint8_t &sys_id) { msg->sys_id() = sys_id; }

	template <class T>
	inline void setMsgSeq(T *msg, const uint8_t &seq) { msg->seq() = seq; }
@[end if]@

	template <class T>
	inline void setMsgTimestampSample(T *msg, const uint64_t &timestamp_sample) { setMsgTimestampSample_impl(msg, timestamp_sample); }

	/**
	 * @@brief Timesync object ptr.
	 *         This object is used to compuyte and apply the time offsets to the
	 *         messages timestamps.
	 */
	std::shared_ptr<TimeSync> _timesync;

	std::shared_ptr<Transform_Imu> _transform{nullptr};
};
