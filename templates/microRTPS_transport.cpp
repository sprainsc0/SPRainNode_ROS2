
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <sys/socket.h>
#include <cstdlib>
#include <inttypes.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#if defined(__linux__)
#include <linux/serial.h>
#endif /* __linux__ */

#include "microRTPS_transport.h"


/** CRC table for the CRC-16. The poly is 0x8005 (x^16 + x^15 + x^2 + 1) */
uint16_t const crc16_table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

Transport_node::Transport_node(const uint8_t sys_id, const bool debug):
	_rx_buff_pos(0),
	_debug(debug),
	_sys_id(sys_id)
{
}

Transport_node::~Transport_node()
{
}

uint16_t Transport_node::crc16_byte(uint16_t crc, const uint8_t data)
{
	return (crc >> 8) ^ crc16_table[(crc ^ data) & 0xff];
}

uint16_t Transport_node::crc16(uint8_t const *buffer, size_t len)
{
	uint16_t crc = 0;

	while (len--) {
		crc = crc16_byte(crc, *buffer++);
	}

	return crc;
}

ssize_t Transport_node::read(uint8_t *topic_id, char out_buffer[], size_t buffer_len)
{
	if (nullptr == out_buffer || nullptr == topic_id || !fds_OK()) {
		return -1;
	}

	*topic_id = 255;

	ssize_t len = node_read((void *)(_rx_buffer + _rx_buff_pos), sizeof(_rx_buffer) - _rx_buff_pos);

	if (len < 0) {
		int errsv = errno;

		if (errsv && EAGAIN != errsv && ETIMEDOUT != errsv) {

			if (_debug) { printf("\033[0;31m[ micrortps_transport ]\tRead fail %d\033[0m\n", errsv); }

		}

		return len;
	}

	_rx_buff_pos += len;

	// We read some
	size_t header_size = sizeof(struct Header);

	// but not enough
	if (_rx_buff_pos < header_size) {
		return 0;
	}

	uint32_t msg_start_pos = 0;

	for (msg_start_pos = 0; msg_start_pos <= _rx_buff_pos - header_size; ++msg_start_pos) {
		if ('>' == _rx_buffer[msg_start_pos] && memcmp(_rx_buffer + msg_start_pos, ">>>", 3) == 0) {
			break;
		}
	}

	// Start not found
	if (msg_start_pos > (_rx_buff_pos - header_size)) {

		if (_debug) { printf("\033[1;33m[ micrortps_transport ]\t                                (↓↓ %" PRIu32 ")\033[0m\n", msg_start_pos); }

		// All we've checked so far is garbage, drop it - but save unchecked bytes
		memmove(_rx_buffer, _rx_buffer + msg_start_pos, _rx_buff_pos - msg_start_pos);
		_rx_buff_pos -= msg_start_pos;
		return -1;
	}

	// [>,>,>,topic_id,sys_id,seq,payload_length_H,payload_length_L,CRCHigh,CRCLow,payloadStart, ... ,payloadEnd]
	struct Header *header = (struct Header *)&_rx_buffer[msg_start_pos];
	uint32_t payload_len = ((uint32_t)header->payload_len_h << 8) | header->payload_len_l;

	// The received message comes from this system. Discard it.
	// This might happen when:
	//   1. The same UDP port is being used to send a rcv packets or
	//   2. The same topic on the agent is being used for outgoing and incoming data
	if (header->sys_id == _sys_id) {
		// Drop the message and continue with the read buffer
		memmove(_rx_buffer, _rx_buffer + msg_start_pos + 1, _rx_buff_pos - (msg_start_pos + 1));
		_rx_buff_pos -= (msg_start_pos + 1);
		return -1;
	}

	// The message won't fit the buffer.
	if (buffer_len < header_size + payload_len) {
		// Drop the message and continue with the read buffer
		memmove(_rx_buffer, _rx_buffer + msg_start_pos + 1, _rx_buff_pos - (msg_start_pos + 1));
		_rx_buff_pos -= (msg_start_pos + 1);
		return -EMSGSIZE;
	}

	// We do not have a complete message yet
	if (msg_start_pos + header_size + payload_len > _rx_buff_pos) {
		// If there's garbage at the beginning, drop it
		if (msg_start_pos > 0) {

			if (_debug) { printf("\033[1;33m[ micrortps_transport ]\t                                (↓ %" PRIu32 ")\033[0m\n", msg_start_pos); }

			memmove(_rx_buffer, _rx_buffer + msg_start_pos, _rx_buff_pos - msg_start_pos);
			_rx_buff_pos -= msg_start_pos;
		}

		return 0;
	}

	uint16_t read_crc = ((uint16_t)header->crc_h << 8) | header->crc_l;
	uint16_t calc_crc = crc16((uint8_t *)_rx_buffer + msg_start_pos + header_size, payload_len);

	if (read_crc != calc_crc) {

		if (_debug) { printf("\033[0;31m[ micrortps_transport ]\tBad CRC %" PRIu16 " != %" PRIu16 "\t\t(↓ %lu)\033[0m\n", read_crc, calc_crc, (unsigned long)(header_size + payload_len)); }


		// Drop garbage up just beyond the start of the message
		memmove(_rx_buffer, _rx_buffer + (msg_start_pos + 1), _rx_buff_pos);

		// If there is a CRC error, the payload len cannot be trusted
		_rx_buff_pos -= (msg_start_pos + 1);

		len = -1;

	} else {
		// copy message to outbuffer and set other return values
		memmove(out_buffer, _rx_buffer + msg_start_pos + header_size, payload_len);
		*topic_id = header->topic_id;
		len = payload_len + header_size;

		// discard message from _rx_buffer
		_rx_buff_pos -= msg_start_pos + header_size + payload_len;
		memmove(_rx_buffer, _rx_buffer + msg_start_pos + header_size + payload_len, _rx_buff_pos);
	}

	return len;
}

size_t Transport_node::get_header_length()
{
	return sizeof(struct Header);
}

ssize_t Transport_node::write(const uint8_t topic_id, char buffer[], size_t length)
{
	if (!fds_OK()) {
		return -1;
	}

	static struct Header header = {{'>', '>', '>'}, 0u, 0u, 0u, 0u, 0u, 0u, 0u};

	// [>,>,>,topic_id,seq,payload_length,CRCHigh,CRCLow,payload_start, ... ,payload_end]
	uint16_t crc = crc16((uint8_t *)&buffer[sizeof(header)], length);

	header.topic_id = topic_id;
	header.sys_id = _sys_id;
	header.seq = _seq_number++;
	header.payload_len_h = (length >> 8) & 0xff;
	header.payload_len_l = length & 0xff;
	header.crc_h = (crc >> 8) & 0xff;
	header.crc_l = crc & 0xff;

	/* Headroom for header is created in client */
	/* Fill in the header in the same payload buffer to call a single node_write */
	memcpy(buffer, &header, sizeof(header));
	ssize_t len = node_write(buffer, length + sizeof(header));

	if (len != ssize_t(length + sizeof(header))) {
		return len;
	}

	return len + sizeof(header);
}

UART_node::UART_node(const char *uart_name, const uint32_t baudrate,
		     const uint32_t poll_ms, const bool hw_flow_control,
		     const bool sw_flow_control, const uint8_t sys_id,
				 const bool debug):
	Transport_node(sys_id, debug),
	_uart_fd(-1),
	_baudrate(baudrate),
	_poll_ms(poll_ms),
	_hw_flow_control(hw_flow_control),
	_sw_flow_control(sw_flow_control)
{

	if (nullptr != uart_name) {
		strcpy(_uart_name, uart_name);
	}
}

UART_node::~UART_node()
{
	close();
}

int UART_node::init()
{
	// Open a serial port
	_uart_fd = open(_uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart_fd < 0) {

		printf("\033[0;31m[ micrortps_transport ]\tUART transport: Failed to open device: %s (%d)\033[0m\n", _uart_name, errno);

		return -errno;
	}

	// If using shared UART, no need to set it up
	if (_baudrate == 0) {
		_poll_fd[0].fd = _uart_fd;
		_poll_fd[0].events = POLLIN;
		return _uart_fd;
	}

	// Try to set baud rate
	struct termios uart_config;
	int termios_state;

	// Back up the original uart configuration to restore it after exit
	if ((termios_state = tcgetattr(_uart_fd, &uart_config)) < 0) {
		int errno_bkp = errno;

		printf("\033[0;31m[ micrortps_transport ]\tUART transport: ERR GET CONF %s: %d (%d)\n\033[0m", _uart_name, termios_state,
		       errno);

		close();
		return -errno_bkp;
	}

#if defined(__linux__)
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

	uart_config.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE | ECHONL | ICANON | IEXTEN | ISIG);

	// never send SIGTTOU
	uart_config.c_lflag &= ~(TOSTOP);

	// ignore modem control lines
	uart_config.c_cflag |= CLOCAL;

	// 8 bits
	uart_config.c_cflag |= CS8;
#else /* __linux__ */

	// Clear ONLCR flag (which appends a CR for every LF)
	uart_config.c_oflag &= ~ONLCR;
#endif

	// Flow control
	if (_hw_flow_control) {
		// HW flow control
		uart_config.c_cflag |= CRTSCTS;
		uart_config.c_iflag &= ~(IXON | IXOFF | IXANY);
	} else if (_sw_flow_control) {
		// SW flow control
		uart_config.c_cflag &= ~CRTSCTS;
		uart_config.c_lflag |= (IXON | IXOFF | IXANY);
	} else {
		uart_config.c_cflag &= ~CRTSCTS;
		uart_config.c_iflag &= ~(IXON | IXOFF | IXANY);
	}

	// Set baud rate
	speed_t speed;

	if (!baudrate_to_speed(_baudrate, &speed)) {

		printf("\033[0;31m[ micrortps_transport ]\tUART transport: ERR SET BAUD %s: Unsupported _baudrate: %d\n\tsupported examples:\n\t9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 921600, 1000000\033[0m\n",
		       _uart_name, _baudrate);

		close();
		return -EINVAL;
	}

	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		int errno_bkp = errno;

		printf("\033[0;31m[ micrortps_transport ]\tUART transport: ERR SET BAUD %s: %d (%d)\033[0m\n", _uart_name, termios_state,
		       errno);

		close();
		return -errno_bkp;
	}

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		int errno_bkp = errno;

		printf("\033[0;31m[ micrortps_transport ]\tUART transport: ERR SET CONF %s (%d)\033[0m\n", _uart_name, errno);

		close();
		return -errno_bkp;
	}

#if defined(__linux__)
	// For Linux, set high speed polling at the chip level. Since this routine relies on a USB latency
	// change at the chip level it may fail on certain chip sets if their driver does not support this
	// configuration request
	{
		struct serial_struct serial_ctl;

		if (ioctl(_uart_fd, TIOCGSERIAL, &serial_ctl) < 0) {
			printf("\033[0;31m[ micrortps_transport ]\tError while trying to read serial port configuration: %d\033[0m\n", errno);

			if (ioctl(_uart_fd, TCFLSH, TCIOFLUSH) == -1) {
				int errno_bkp = errno;
				printf("\033[0;31m[ protocol__splitter ]\tCould not flush terminal\033[0m\n");
				close();
				return -errno_bkp;
			}
		}

		serial_ctl.flags |= ASYNC_LOW_LATENCY;

		if (ioctl(_uart_fd, TIOCSSERIAL, &serial_ctl) < 0) {
			int errno_bkp = errno;
			printf("\033[0;31m[ micrortps_transport ]\tError while trying to write serial port latency: %d\033[0m\n", errno);
			close();
			return -errno_bkp;
		}
	}
#endif /* __linux__ */

	char aux[64];
	bool flush = false;

	while (0 < ::read(_uart_fd, (void *)&aux, 64)) {
		flush = true;

		usleep(1000);

	}

	if (flush) {

		if (_debug) { printf("[ micrortps_transport ]\tUART transport: Flush\n"); }


	} else {

		if (_debug) { printf("[ micrortps_transport ]\tUART transport: No flush\n"); }

	}

	_poll_fd[0].fd = _uart_fd;
	_poll_fd[0].events = POLLIN;

	return _uart_fd;
}

bool UART_node::fds_OK()
{
	return (-1 != _uart_fd);
}

uint8_t UART_node::close()
{
	if (-1 != _uart_fd) {

		printf("\033[1;33m[ micrortps_transport ]\tClosed UART.\n\033[0m");

		::close(_uart_fd);
		_uart_fd = -1;
		memset(&_poll_fd, 0, sizeof(_poll_fd));
	}

	return 0;
}

ssize_t UART_node::node_read(void *buffer, size_t len)
{
	if (nullptr == buffer || !fds_OK()) {
		return -1;
	}

	ssize_t ret = 0;
	int r = poll(_poll_fd, 1, _poll_ms);

	if (r == 1 && (_poll_fd[0].revents & POLLIN)) {
		ret = ::read(_uart_fd, buffer, len);
	}

	return ret;
}

ssize_t UART_node::node_write(void *buffer, size_t len)
{
	if (nullptr == buffer || !fds_OK()) {
		return -1;
	}

	return ::write(_uart_fd, buffer, len);
}

bool UART_node::baudrate_to_speed(uint32_t bauds, speed_t *speed)
{
#ifndef B460800
#define B460800 460800
#endif

#ifndef B500000
#define B500000 500000
#endif

#ifndef B921600
#define B921600 921600
#endif

#ifndef B1000000
#define B1000000 1000000
#endif

#ifndef B1500000
#define B1500000 1500000
#endif

#ifndef B2000000
#define B2000000 2000000
#endif

	switch (bauds) {
	case 0:      *speed = B0;		break;

	case 50:     *speed = B50;		break;

	case 75:     *speed = B75;		break;

	case 110:    *speed = B110;		break;

	case 134:    *speed = B134;		break;

	case 150:    *speed = B150;		break;

	case 200:    *speed = B200;		break;

	case 300:    *speed = B300;		break;

	case 600:    *speed = B600;		break;

	case 1200:   *speed = B1200;		break;

	case 1800:   *speed = B1800;		break;

	case 2400:   *speed = B2400;		break;

	case 4800:   *speed = B4800;		break;

	case 9600:   *speed = B9600;		break;

	case 19200:  *speed = B19200;		break;

	case 38400:  *speed = B38400;		break;

	case 57600:  *speed = B57600;		break;

	case 115200: *speed = B115200;		break;

	case 230400: *speed = B230400;		break;

	case 460800: *speed = B460800;		break;

	case 500000: *speed = B500000;		break;

	case 921600: *speed = B921600;		break;

	case 1000000: *speed = B1000000;	break;

	case 1500000: *speed = B1500000;	break;

	case 2000000: *speed = B2000000;	break;

#ifdef B3000000
	case 3000000: *speed = B3000000;	break;

#endif
#ifdef B3500000
	case 3500000: *speed = B3500000;	break;

#endif
#ifdef B4000000
	case 4000000: *speed = B4000000;	break;

#endif
	default:
		return false;
	}

	return true;
}

UDP_node::UDP_node(const char *udp_ip, uint16_t udp_port_recv,
		   uint16_t udp_port_send, const uint8_t sys_id, const bool debug):
	Transport_node(sys_id, debug),
	_sender_fd(-1),
	_receiver_fd(-1),
	_udp_port_recv(udp_port_recv),
	_udp_port_send(udp_port_send)
{
	if (nullptr != udp_ip) {
		strcpy(_udp_ip, udp_ip);
	}
}

UDP_node::~UDP_node()
{
	close();
}

int UDP_node::init()
{
	if (0 > init_receiver(_udp_port_recv) || 0 > init_sender(_udp_port_send)) {
		return -1;
	}

	return 0;
}

bool UDP_node::fds_OK()
{
	return (-1 != _sender_fd && -1 != _receiver_fd);
}

int UDP_node::init_receiver(uint16_t udp_port)
{
	// udp socket data
	memset((char *)&_receiver_inaddr, 0, sizeof(_receiver_inaddr));
	_receiver_inaddr.sin_family = AF_INET;
	_receiver_inaddr.sin_port = htons(udp_port);
	_receiver_inaddr.sin_addr.s_addr = htonl(INADDR_ANY);

	if ((_receiver_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {

		printf("\033[0;31m[ micrortps_transport ]\tUDP transport: Create socket failed\033[0m\n");

		return -1;
	}

	printf("[ micrortps_transport ]\tUDP transport: Trying to connect...\n");


	if (bind(_receiver_fd, (struct sockaddr *)&_receiver_inaddr, sizeof(_receiver_inaddr)) < 0) {

		printf("\033[0;31m[ micrortps_transport ]\tUDP transport: Bind failed\033[0m\n");

		return -1;
	}

	printf("[ micrortps_transport ]\tUDP transport: Connected to server!\n\n");

	return 0;
}

int UDP_node::init_sender(uint16_t udp_port)
{

	if ((_sender_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {

		printf("\033[0;31m[ micrortps_transport ]\tUDP transport: Create socket failed\033[0m\n");

		return -1;
	}

	memset((char *) &_sender_outaddr, 0, sizeof(_sender_outaddr));
	_sender_outaddr.sin_family = AF_INET;
	_sender_outaddr.sin_port = htons(udp_port);

	if (inet_aton(_udp_ip, &_sender_outaddr.sin_addr) == 0) {

		printf("\033[0;31m[ micrortps_transport ]\tUDP transport: inet_aton() failed\033[0m\n");

		return -1;
	}

	return 0;
}

uint8_t UDP_node::close()
{

	if (_sender_fd != -1) {

		printf("\033[1;33m[ micrortps_transport ]\tUDP transport: Closed sender socket!\033[0m\n");

		shutdown(_sender_fd, SHUT_RDWR);
		::close(_sender_fd);
		_sender_fd = -1;
	}

	if (_receiver_fd != -1) {

		printf("\033[1;33m[ micrortps_transport ]\tUDP transport: Closed receiver socket!\033[0m\n");

		shutdown(_receiver_fd, SHUT_RDWR);
		::close(_receiver_fd);
		_receiver_fd = -1;
	}
	return 0;
}

ssize_t UDP_node::node_read(void *buffer, size_t len)
{
	if (nullptr == buffer || !fds_OK()) {
		return -1;
	}

	ssize_t ret = 0;
	// Blocking call
	static socklen_t addrlen = sizeof(_receiver_outaddr);
	ret = recvfrom(_receiver_fd, buffer, len, 0, (struct sockaddr *)&_receiver_outaddr, &addrlen);
	return ret;
}

ssize_t UDP_node::node_write(void *buffer, size_t len)
{
	if (nullptr == buffer || !fds_OK()) {
		return -1;
	}

	ssize_t ret = 0;
	ret = sendto(_sender_fd, buffer, len, 0, (struct sockaddr *)&_sender_outaddr, sizeof(_sender_outaddr));
	return ret;
}

PIPE_node::PIPE_node(const char *pipe_ros, const char *pipe_fcu, const uint32_t poll_ms,
		 const uint8_t sys_id, const bool debug):
	Transport_node(sys_id, debug),
	_to_ros_fd(-1),
	_to_fcu_fd(-1),
	_poll_ms(poll_ms)
{

	if (nullptr != pipe_ros) {
		strcpy(_pipe_ros_name, pipe_ros);
	}

	if (nullptr != pipe_fcu) {
		strcpy(_pipe_fcu_name, pipe_fcu);
	}
}

PIPE_node::~PIPE_node()
{
	close();
}

int PIPE_node::init()
{
	if(-1 == access(_pipe_ros_name, F_OK)) {
        int ret = mkfifo(_pipe_ros_name,0777);
        if(ret == -1) {
            printf("\033[1;33m[ micrortps_transport ]\tPIPE transport: pipe %s mkfifo error %d \033[0m\n", _pipe_ros_name, ret);
            return -1;
        }
    }

	if(-1 == access(_pipe_fcu_name, F_OK)) {
        int ret = mkfifo(_pipe_fcu_name,0777);
        if(ret == -1) {
            printf("\033[1;33m[ micrortps_transport ]\tPIPE transport: pipe %s mkfifo error %d \033[0m\n", _pipe_fcu_name, ret);
            return -1;
        }
    }

	// Open a pipe port
	_to_fcu_fd = open(_pipe_fcu_name, O_WRONLY);
	if (_to_fcu_fd < 0) {

		printf("\033[0;31m[ micrortps_transport ]\tPIPE transport: Failed to open pipe: %s (%d)\033[0m\n", _pipe_fcu_name, errno);
		return -errno;
	}

	_to_ros_fd = open(_pipe_ros_name, O_RDONLY);
	if (_to_ros_fd < 0) {

		printf("\033[0;31m[ micrortps_transport ]\tPIPE transport: Failed to open pipe: %s (%d)\033[0m\n", _pipe_ros_name, errno);
		return -errno;
	}

	_poll_fd[0].fd = _to_ros_fd;
	_poll_fd[0].events = POLLIN;

	return _to_ros_fd;
}

bool PIPE_node::fds_OK()
{
	return (-1 != _to_ros_fd) && (-1 != _to_fcu_fd);
}

uint8_t PIPE_node::close()
{
	if (-1 != _to_ros_fd) {

		printf("\033[1;33m[ micrortps_transport ]\tClosed recv pipe.\n\033[0m");

		::close(_to_ros_fd);
		_to_ros_fd = -1;
		memset(&_poll_fd, 0, sizeof(_poll_fd));
	}

	if (-1 != _to_fcu_fd) {

		printf("\033[1;33m[ micrortps_transport ]\tClosed send pipe.\n\033[0m");

		::close(_to_fcu_fd);
		_to_fcu_fd = -1;
	}

	return 0;
}

ssize_t PIPE_node::node_read(void *buffer, size_t len)
{
	if (nullptr == buffer || !fds_OK()) {
		return -1;
	}

	ssize_t ret = 0;
	int r = poll(_poll_fd, 1, _poll_ms);

	if (r > 0 && (_poll_fd[0].revents & POLLIN)) {
		ret = ::read(_to_ros_fd, buffer, len);
	}

	return ret;
}

ssize_t PIPE_node::node_write(void *buffer, size_t len)
{
	if (nullptr == buffer || !fds_OK()) {
		return -1;
	}

	return ::write(_to_fcu_fd, buffer, len);
}