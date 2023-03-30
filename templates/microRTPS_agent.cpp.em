@###############################################
@#
@# EmPy template for generating microRTPS_agent.cpp file
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - spec (msggen.MsgSpec) Parsed specification of the .msg file
@#  - ros2_distro (List[str]) ROS2 distro name
@###############################################
@{
import genmsg.msgs
from generate_uorb_topic_files import MsgScope # this is in Tools/

try:
    ros2_distro = ros2_distro[0].decode("utf-8")
except AttributeError:
    ros2_distro = ros2_distro[0]

send_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND]
recv_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.RECEIVE]
}@

#include <thread>
#include <atomic>
#include <getopt.h>
#include <unistd.h>
#include <poll.h>
#include <chrono>
#include <ctime>
#include <csignal>
#include <termios.h>
#include <condition_variable>
#include <queue>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>
#include <fastcdr/exceptions/Exception.h>
#include <fastrtps/Domain.h>

#include "microRTPS_transport.h"
#include "microRTPS_timesync.h"
#include "RtpsTopics.h"

@[if ros2_distro]@
#include <rclcpp/rclcpp.hpp>
@[end if]@

// Default values
#define SLEEP_US          2000
#define MAX_SLEEP_US      1000000
#define MAX_DATA_RATE     10000000
#define PIPE_ROS          "/tmp/pipe2ros"
#define PIPE_FCU          "/tmp/pipe2fcu"
#define POLL_MS           10
#define MAX_POLL_MS       1000

using namespace eprosima;
using namespace eprosima::fastrtps;

volatile sig_atomic_t running = 1;
std::unique_ptr<Transport_node> transport_node;
std::unique_ptr<RtpsTopics> topics;
uint32_t total_sent = 0, sent = 0;

struct options {
	char pipe_ros[64] = PIPE_ROS;
	char pipe_fcu[64] = PIPE_FCU;
	int sleep_us = SLEEP_US;
	int poll_ms = POLL_MS;
	bool verbose_debug = false;
	std::string ns = "";
} _options;

static void usage(const char *name)
{
	printf("usage: %s [options]\n\n"
		   "  -c <pipe>               PIPE name to ROS2\n"
		   "  -u <pipe>               PIPE name to FCU\n"
	       "  -n <namespace>          Topics namespace. Identifies the vehicle in a multi-agent network\n"
	       "  -o <poll-ms>            polling timeout in milliseconds. Defaults to 1ms\n"
	       "  -v <increase-verbosity> Add more verbosity\n"
	       "  -w <sleep-time-us>      Iteration time for data publishing to the DDS world, in microseconds.\n"
	       "                           Defaults to 1us\n"
	       "     <ros-args>           (ROS2 only) Allows to pass arguments to the timesync ROS2 node.\n"
	       "                           Currently used for setting the usage of simulation time by the node using\n"
	       "                           '--ros-args -p use_sim_time:=true'\n",
	       name);
}

static int parse_options(int argc, char **argv)
{
	static const struct option options[] = {
		{"pipe_ros", required_argument, NULL, 'c'},
		{"pipe_fcu", required_argument, NULL, 'u'},
		{"namespace", required_argument, NULL, 'n'},
		{"poll-ms", required_argument, NULL, 'o'},
		{"increase-verbosity", no_argument, NULL, 'v'},
		{"sleep-time-us", required_argument, NULL, 'w'},
		{"ros-args", required_argument, NULL, 0},
		{"help", no_argument, NULL, 'h'},
		{NULL, 0, NULL, 0}};

	int ch;

	while ((ch = getopt_long(argc, argv, "t:d:w:b:o:r:s:i:fghvn:", options, nullptr)) >= 0) {
		switch (ch) {
		case 'c': if (nullptr != optarg) strcpy(_options.pipe_ros, optarg); break;

		case 'u': if (nullptr != optarg) strcpy(_options.pipe_fcu, optarg); break;

		case 'w': _options.sleep_us        = strtoul(optarg, nullptr, 10);  break;

		case 'o': _options.poll_ms         = strtol(optarg, nullptr, 10);   break;

		case 'h': usage(argv[0]); exit(0);                            	    break;

		case 'v': _options.verbose_debug = true;                            break;

		case 'n': if (nullptr != optarg) _options.ns = std::string(optarg) + "/"; break;

		default:
@[if ros2_distro]@
			break;
@[else]@
			usage(argv[0]);
			return -1;
@[end if]@
		}
	}

	if (_options.poll_ms < POLL_MS) {
		_options.poll_ms = POLL_MS;
		printf("\033[1;33m[   micrortps_agent   ]\tPoll timeout too low. Using %d ms instead\033[0m\n", POLL_MS);
	} else if (_options.poll_ms > MAX_POLL_MS) {
		_options.poll_ms = MAX_POLL_MS;
		printf("\033[1;33m[   micrortps_agent   ]\tPoll timeout too high. Using %d ms instead\033[0m\n", MAX_POLL_MS);
	}

	if (_options.sleep_us > MAX_SLEEP_US) {
		_options.sleep_us = MAX_SLEEP_US;
		printf("\033[1;33m[   micrortps_agent   ]\tPublishing iteration cycle too slow. Using %d us instead\033[0m\n", MAX_SLEEP_US);
	}

	return 0;
}

@[if recv_topics]@
std::atomic<bool> exit_sender_thread(false);
std::condition_variable t_send_queue_cv;
std::mutex t_send_queue_mutex;
std::queue<uint8_t> t_send_queue;

void t_send(void *)
{
	char data_buffer[BUFFER_SIZE] = {};
	uint32_t length = 0;
	uint8_t topic_ID = 255;

	while (running && !exit_sender_thread) {
		std::unique_lock<std::mutex> lk(t_send_queue_mutex);

		while (t_send_queue.empty() && !exit_sender_thread) {
			t_send_queue_cv.wait(lk);
		}

		topic_ID = t_send_queue.front();
		t_send_queue.pop();
		lk.unlock();

		size_t header_length = transport_node->get_header_length();
		/* make room for the header to fill in later */
		eprosima::fastcdr::FastBuffer cdrbuffer(&data_buffer[header_length], sizeof(data_buffer) - header_length);
		eprosima::fastcdr::Cdr scdr(cdrbuffer);

		if (!exit_sender_thread) {
			if (topics->getMsg(topic_ID, scdr)) {
				length = scdr.getSerializedDataLength();

				if (0 < (length = transport_node->write(topic_ID, data_buffer, length))) {
					total_sent += length;
					++sent;
				}
			}
		}
	}
}
@[end if]@

void signal_handler(int signum)
{
	printf("\n\033[1;33m[   micrortps_agent   ]\tInterrupt signal (%d) received.\033[0m\n", signum);
	running = 0;
	transport_node->close();
}

int main(int argc, char **argv)
{
	printf("\033[0;37m--- MicroRTPS Agent ---\033[0m\n");

	if (-1 == parse_options(argc, argv)) {
		printf("\033[1;33m[   micrortps_agent   ]\tEXITING...\033[0m\n");
		return -1;
	}

	topics = std::make_unique<RtpsTopics>();

@[if ros2_distro]@
	// Initialize communications via the rmw implementation and set up a global signal handler.
	rclcpp::init(argc, argv, rclcpp::InitOptions());
@[end if]@

	// register signal SIGINT and signal handler
	signal(SIGINT, signal_handler);

	printf("[   micrortps_agent   ]\tStarting link...\n");

	const char* localhost_only = std::getenv("ROS_LOCALHOST_ONLY");
	const char* rmw_implementation = std::getenv("RMW_IMPLEMENTATION");
	const char* ros_distro = std::getenv("ROS_DISTRO");

	
	printf("[   micrortps_agent   ]\tRmw implementation:%s\n", rmw_implementation);
	printf("[   micrortps_agent   ]\tRos_distro:%s\n", ros_distro);

	if (localhost_only && strcmp(localhost_only, "1") == 0) {
		printf("[   micrortps_agent   ]\tUsing only the localhost network...\n");
	}

	/**
	 * Set the system ID to Mission Computer, in order to identify the agent side
	 *
	 * Note: theoretically a multi-agent system is possible, but this would require
	 * adjustments in the way the timesync is done (would have to create a timesync
	 * instance per agent). Keeping it contained for a 1:1 link for now is reasonable.
	 */
	const uint8_t sys_id = static_cast<uint8_t>(MicroRtps::System::MISSION_COMPUTER);
	
	transport_node = std::make_unique<PIPE_node>(_options.pipe_ros, _options.pipe_fcu, _options.poll_ms,
						sys_id, _options.verbose_debug);
	printf("[   micrortps_agent   ]\tPIPE transport: pipe_ros: %s; pipe_fcu: %s; poll: %dms; \n",
			_options.pipe_ros, _options.pipe_fcu, _options.poll_ms);
		
	if (0 > transport_node->init()) {
		printf("\033[0;37m[   micrortps_agent   ]\tEXITING...\033[0m\n");
		return -1;
	}

	sleep(1);

@[if send_topics]@
	char data_buffer[BUFFER_SIZE] = {};
	int received = 0, loop = 0;
	int length = 0, total_read = 0;
	bool receiving = false;
	uint8_t topic_ID = 255;
	std::chrono::time_point<std::chrono::steady_clock> start, end;
@[end if]@

	// Init timesync
	topics->set_timesync(std::make_shared<TimeSync>(_options.verbose_debug));

@[if recv_topics]@
	topics->init(&t_send_queue_cv, &t_send_queue_mutex, &t_send_queue, _options.ns);
@[end if]@

	running = true;
@[if recv_topics]@
	std::thread sender_thread(t_send, nullptr);
@[end if]@

	while (running) {
@[if send_topics]@
		++loop;
		if (!receiving) { start = std::chrono::steady_clock::now(); }

		// Publish messages received from UART
		if (0 < (length = transport_node->read(&topic_ID, data_buffer, BUFFER_SIZE))) {
			topics->publish(topic_ID, data_buffer, sizeof(data_buffer));
			++received;
			total_read += length;
			receiving = true;
			end = std::chrono::steady_clock::now();
		}
@[else]@
		usleep(_options.sleep_us);
@[end if]@
	}

@[if recv_topics]@
	exit_sender_thread = true;
	t_send_queue_cv.notify_one();
	sender_thread.join();

	std::chrono::duration<double> elapsed_secs = end - start;
	if (received > 0) {
		printf("[   micrortps_agent   ]\tRECEIVED: %d messages - %d bytes; %d LOOPS - %.03f seconds - %.02fKB/s\n",
		       received, total_read, loop, elapsed_secs.count(), static_cast<double>(total_read) / (1000 * elapsed_secs.count()));
	}
@[end if]@
@[if recv_topics]@
	if (sent > 0) {
		printf("[   micrortps_agent   ]\tSENT:     %lu messages - %lu bytes\n", static_cast<unsigned long>(sent),
		       static_cast<unsigned long>(total_sent));
	}
@[end if]@

@[if ros2_distro]@
	rclcpp::shutdown();
@[end if]@
	transport_node.reset();
	topics.reset();

	return 0;
}
