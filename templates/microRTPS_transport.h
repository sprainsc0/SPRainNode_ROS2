

#pragma once

#include <cstring>
#include <arpa/inet.h>
#include <poll.h>
#include <termios.h>

#define BUFFER_SIZE 4096
#define DEFAULT_UART "/dev/ttyACM0"

namespace MicroRtps {
	enum class System {
		FMU,
		MISSION_COMPUTER
	};
}

class Transport_node
{
public:
	Transport_node(const uint8_t sys_id, const bool debug);
	virtual ~Transport_node();

	virtual int init() {return 0;}
	virtual uint8_t close() {return 0;}
	ssize_t read(uint8_t *topic_id, char out_buffer[], size_t buffer_len);

	/**
	 * write a buffer
	 * @param topic_id
	 * @param buffer buffer to write: it must leave get_header_length() bytes free at the beginning. This will be
	 *               filled with the header. length does not include get_header_length(). So buffer looks like this:
	 *                -------------------------------------------------
	 *               | header (leave free)          | payload data     |
	 *               | get_header_length() bytes    | length bytes     |
	 *                -------------------------------------------------
	 * @param length buffer length excluding header length
	 * @return length on success, <0 on error
	 */
	ssize_t write(const uint8_t topic_id, char buffer[], size_t length);

	/** Get the Length of struct Header to make headroom for the size of struct Header along with payload */
	size_t get_header_length();

private:
	struct __attribute__((packed)) Header {
		char marker[3];
		uint8_t topic_id;
		uint8_t sys_id;
		uint8_t seq;
		uint8_t payload_len_h;
		uint8_t payload_len_l;
		uint8_t crc_h;
		uint8_t crc_l;
	};

protected:
	virtual ssize_t node_read(void *buffer, size_t len) = 0;
	virtual ssize_t node_write(void *buffer, size_t len) = 0;
	virtual bool fds_OK() = 0;
	uint16_t crc16_byte(uint16_t crc, const uint8_t data);
	uint16_t crc16(uint8_t const *buffer, size_t len);

	uint32_t _rx_buff_pos;
	char _rx_buffer[BUFFER_SIZE]{};

	bool _debug;

	uint8_t _sys_id;
	uint8_t _seq_number{0};
};

class UART_node: public Transport_node
{
public:
	UART_node(const char *uart_name, const uint32_t baudrate,
		  const uint32_t poll_ms, const bool hw_flow_control,
		  const bool sw_flow_control, const uint8_t sys_id,
			const bool debug);
	virtual ~UART_node();

	int init();
	uint8_t close();

protected:
	ssize_t node_read(void *buffer, size_t len);
	ssize_t node_write(void *buffer, size_t len);
	bool fds_OK();
	bool baudrate_to_speed(uint32_t bauds, speed_t *speed);

	int _uart_fd;
	char _uart_name[64]{};
	uint32_t _baudrate;
	uint32_t _poll_ms;
	bool _hw_flow_control{false};
	bool _sw_flow_control{false};
	struct pollfd _poll_fd[1]{};
};

class UDP_node: public Transport_node
{
public:
	UDP_node(const char *udp_ip, uint16_t udp_port_recv, uint16_t udp_port_send,
		 const uint8_t sys_id, const bool debug);
	virtual ~UDP_node();

	int init();
	uint8_t close();

protected:
	int init_receiver(uint16_t udp_port);
	int init_sender(uint16_t udp_port);
	ssize_t node_read(void *buffer, size_t len);
	ssize_t node_write(void *buffer, size_t len);
	bool fds_OK();

	int _sender_fd;
	int _receiver_fd;
	char _udp_ip[16]{};
	uint16_t _udp_port_recv;
	uint16_t _udp_port_send;
	struct sockaddr_in _sender_outaddr;
	struct sockaddr_in _receiver_inaddr;
	struct sockaddr_in _receiver_outaddr;
};

class PIPE_node: public Transport_node
{
public:
	PIPE_node(const char *pipe_ros, const char *pipe_fcu, const uint32_t poll_ms,
		 const uint8_t sys_id, const bool debug);
	virtual ~PIPE_node();

	int init();
	uint8_t close();

protected:
	ssize_t node_read(void *buffer, size_t len);
	ssize_t node_write(void *buffer, size_t len);
	bool fds_OK();

	int _to_ros_fd;
	int _to_fcu_fd;
	uint32_t _poll_ms;

	char _pipe_ros_name[64]{};
	char _pipe_fcu_name[64]{};
	
	struct pollfd _poll_fd[1]{};
};
