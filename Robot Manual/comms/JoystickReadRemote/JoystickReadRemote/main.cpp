// SPDX-License-Identifier: MIT
/*
 * Copyright © 2013 Red Hat, Inc.
 */

#include <cassert>
#include <cerrno>
#include <fcntl.h>
#include <linux/input.h>
#include <cstdio>
#include <cstring>
#include <cstdint>  // for uint8_t
#include <sys/stat.h>
#include <unistd.h> // for sleep()
#include <sys/types.h>

 //UDP Libs
#include <bits/stdc++.h> 
#include <stdlib.h> 
//#include <unistd.h> 
#include <string.h> 
//#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h>

// Multi threading
#include <iostream>
#include <thread>
#include <mutex>
#include <csignal>
#include <chrono>


extern "C" {
#include <libevdev-1.0/libevdev/libevdev.h>
}

// Fix for E0040 and E0415 errors
// Initialize the `joystickData` structure properly and ensure the identifier is set correctly.

struct joystickData {
	uint16_t identifier;
	int8_t LX;
	int8_t LY;
	int8_t RX;
	int8_t RY;
	uint8_t THR;
	uint8_t BRK;
	uint16_t BTN;
	uint8_t MISC;
	uint8_t DPAD;
	uint8_t BAT;
	uint8_t SUM;
	uint8_t INC;
};

// Create an instance of `joystickData` and initialize its fields
joystickData joystick = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
// Mutex for thread safety
std::mutex joystickMutex;

// UDP Setting
#define PORT		"1412" // Ganti dengan port yang sesuai
#define SERVER_IP	"192.25.2.1" // Ganti dengan IP server yang sesuai
#define INTERFACE_IP	"192.25.2.2" 
#define MAXLINE		128 

std::atomic<bool> running(true); // digunakan oleh semua thread

uint8_t crc8(const uint8_t* data, size_t length, uint8_t crc = 0x00) {
	for (size_t i = 0; i < length; ++i) {
		crc ^= data[i];
		for (int j = 0; j < 8; ++j) {
			if (crc & 0x80)
				crc = (crc << 1) ^ 0x07;
			else
				crc <<= 1;
		}
	}
	return crc;
}

void generateChecksum(joystickData& joy) {
	const uint8_t* start = reinterpret_cast<const uint8_t*>(&joy.LX);
	const size_t length = offsetof(joystickData, SUM) - offsetof(joystickData, LX);

	joy.SUM = crc8(start, length);
}

//bool verifyChecksum(const joystickData& joy) {
//	const uint8_t* start = reinterpret_cast<const uint8_t*>(&joy.LX);
//	const size_t length = offsetof(joystickData, SUM) - offsetof(joystickData, LX);
//
//	uint8_t sum = 0;
//	for (size_t i = 0; i < length; ++i) {
//		sum += start[i];
//	}
//
//	return sum == joy.SUM;
//}

void handle_sigint(int signum) {
	running = false;
	printf("Caught signal %d, exiting...\n", signum);
}

static void
UDPsetupClient(int& sockfd, struct sockaddr_in& servaddr, const char* server_ip, uint16_t port) {
	// 1. Buat socket UDP
	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd < 0) {
		fprintf(stderr, "Error IN: UDPsetupClient");
		return;
	}

	// 2. Siapkan alamat server
	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(port);
	inet_pton(AF_INET, server_ip, &servaddr.sin_addr);
}
	
static void
UDPsendJoysData(int sockfd, const struct sockaddr_in& servaddr, joystickData* data) {
	// 3. Kirim data joystick ke server
	static uint8_t incData = 0;
	switch (incData)
	{
	case 255: // Reset INC field if it reaches 255
		incData = 0;
		break;
	default:
		incData++; // Increment INC field
		break;
	}

	data->INC = incData;

	ssize_t  n = sendto(sockfd, data, sizeof(joystickData), MSG_CONFIRM,
		(const struct sockaddr*)&servaddr, sizeof(servaddr));
	if (n < 0) {
		fprintf(stderr, "sendto failed: %s\n", strerror(errno));
	}
	else {
		//printf("Sent %zd bytes\n", n);
	}
}

static void
UDPthread(const char* port, const char* serverIP) {
	try {
		// UDP Global Variables
		int sockfd;
		struct sockaddr_in servaddr;
		joystickData localJoystick;

		uint16_t _port = static_cast<uint16_t>(std::stoul(port));
		UDPsetupClient(sockfd, servaddr, serverIP, _port);
		while (running)
		{
			{
				std::lock_guard<std::mutex> lock(joystickMutex);
				localJoystick = joystick; // Copy the joystick data safely
			}
			UDPsendJoysData(sockfd, servaddr, &localJoystick);
			std::this_thread::sleep_for(std::chrono::milliseconds(1)); // interval pengiriman
		}
		close(sockfd);
		printf("UDP thread exited.\n");
	}
	catch (const std::exception& e) {
		fprintf(stderr, "[UDPThread Exception] %s\n", e.what());
	}

}

static bool
try_open_device(const char* file, int& fd, struct libevdev*& dev) {
	fd = open(file, O_RDONLY | O_NONBLOCK);
	if (fd < 0) {
		perror("open failed");
		return false;
	}

	int rc = libevdev_new_from_fd(fd, &dev);
	if (rc < 0) {
		fprintf(stderr, "libevdev init failed (%s)\n", strerror(-rc));
		close(fd);
		return false;
	}

	printf("[Reconnected] Device: %s\n", libevdev_get_name(dev));
	return true;
}

static void
print_abs_bits(struct libevdev* dev, int axis)
{
	const struct input_absinfo* abs;

	if (!libevdev_has_event_code(dev, EV_ABS, axis))
		return;

	abs = libevdev_get_abs_info(dev, axis);

	printf("	Value	%6d\n", abs->value);
	printf("	Min	%6d\n", abs->minimum);
	printf("	Max	%6d\n", abs->maximum);
	if (abs->fuzz)
		printf("	Fuzz	%6d\n", abs->fuzz);
	if (abs->flat)
		printf("	Flat	%6d\n", abs->flat);
	if (abs->resolution)
		printf("	Resolution	%6d\n", abs->resolution);
}

static void
print_code_bits(struct libevdev* dev, unsigned int type, unsigned int max)
{
	for (unsigned int i = 0; i <= max; i++) {
		if (!libevdev_has_event_code(dev, type, i))
			continue;

		printf("    Event code %i (%s)\n", i, libevdev_event_code_get_name(type, i));
		if (type == EV_ABS)
			print_abs_bits(dev, i);
	}
}

static void
print_bits(struct libevdev* dev)
{
	printf("Supported events:\n");

	for (unsigned int i = 0; i <= EV_MAX; i++) {
		if (libevdev_has_event_type(dev, i))
			printf("  Event type %d (%s)\n", i, libevdev_event_type_get_name(i));

		switch (i) {
		case EV_KEY:
			print_code_bits(dev, EV_KEY, KEY_MAX);
			break;
		case EV_REL:
			print_code_bits(dev, EV_REL, REL_MAX);
			break;
		case EV_ABS:
			print_code_bits(dev, EV_ABS, ABS_MAX);
			break;
		case EV_LED:
			print_code_bits(dev, EV_LED, LED_MAX);
			break;
		}
	}
}

static void
print_props(struct libevdev* dev)
{
	printf("Properties:\n");

	for (unsigned int i = 0; i <= INPUT_PROP_MAX; i++) {
		if (libevdev_has_property(dev, i))
			printf("  Property type %d (%s)\n", i, libevdev_property_get_name(i));
	}
}

static void
printJoystickData(const joystickData* js) {
	printf("LX:%4d LY:%4d RX:%4d RY:%4d THR:%3u BRK:%3u BTN:%4u MISC:%1u DPAD:%1u BAT:%3u SUM:%3u\n",
		js->LX, js->LY, js->RX, js->RY,
		js->THR, js->BRK,
		js->BTN, js->MISC, js->DPAD,
		js->BAT, js->SUM);
}

static void
parseEvent(struct input_event* ev, joystickData* data)
{
	switch (ev->type)
	{
	case EV_KEY:
		switch (ev->code)
		{
		case BTN_SOUTH:
			if (ev->value)
				data->BTN |= 1; // Use '->' to access members of the pointer  
			else
				data->BTN &= ~1;
			break;

		case BTN_EAST:
			if (ev->value)
				data->BTN |= 2;
			else
				data->BTN &= ~2;
			break;

		case BTN_WEST:
			if (ev->value)
				data->BTN |= 4;
			else
				data->BTN &= ~4;
			break;

		case BTN_NORTH:
			if (ev->value)
				data->BTN |= 8;
			else
				data->BTN &= ~8;
			break;

		case BTN_TL:
			if (ev->value)
				data->BTN |= 16;
			else
				data->BTN &= ~16;
			break;

		case BTN_TR:
			if (ev->value)
				data->BTN |= 32;
			else
				data->BTN &= ~32;
			break;

		case BTN_TL2:
			if (ev->value)
				data->BTN |= 64;
			else
				data->BTN &= ~64;
			break;

		case BTN_TR2:
			if (ev->value)
				data->BTN |= 128;
			else
				data->BTN &= ~128;
			break;

		case BTN_THUMBL:
			if (ev->value)
				data->BTN |= 256;
			else
				data->BTN &= ~256;
			break;

		case BTN_THUMBR:
			if (ev->value)
				data->BTN |= 512;
			else
				data->BTN &= ~512;
			break;

		case BTN_MODE:
			if (ev->value)
				data->MISC |= 1;
			else
				data->MISC &= ~1;
			break;

		case BTN_SELECT:
			if (ev->value)
				data->MISC |= 2;
			else
				data->MISC &= ~2;
			break;

		case BTN_START:
			if (ev->value)
				data->MISC |= 4;
			else
				data->MISC &= ~4;
			break;

		default:
			break;
		}

		break;
	case EV_ABS:
		switch (ev->code) {
		case ABS_X:
			data->LX = static_cast<int8_t>(ev->value - 128);
			break;
		case ABS_Y:
			data->LY = static_cast<int8_t>(ev->value - 128);
			break;
		case ABS_RX:
			data->RX = static_cast<int8_t>(ev->value - 128);
			break;
		case ABS_RY:
			data->RY = static_cast<int8_t>(ev->value - 128);
			break;

		case ABS_RZ:
			data->THR = static_cast<uint8_t>(ev->value);
			break;
		case ABS_Z:
			data->BRK = static_cast<uint8_t>(ev->value);
			break;
		case ABS_HAT0Y:
			switch (ev->value) {
			case -1:
				data->DPAD |= 1;
				break;
			case 1:
				data->DPAD |= 2;
				break;
			default:
				data->DPAD &= ~1;
				data->DPAD &= ~2;
				break;
			}
			break;
		case ABS_HAT0X:
			switch (ev->value) {
			case -1:
				data->DPAD |= 8;
				break;
			case 1:
				data->DPAD |= 4;
				break;
			default:
				data->DPAD &= ~4;
				data->DPAD &= ~8;
				break;
			}
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static int
print_event(struct input_event* ev)
{
	if (ev->type == EV_SYN)
		printf("Event: time %ld.%06ld, ++++++++++++++++++++ %s +++++++++++++++\n",
			ev->input_event_sec,
			ev->input_event_usec,
			libevdev_event_type_get_name(ev->type));
	else {

		printf("Event: time %ld.%06ld, type %d (%s), code %d (%s), value %d\n",
			ev->input_event_sec,
			ev->input_event_usec,
			ev->type,
			libevdev_event_type_get_name(ev->type),
			ev->code,
			libevdev_event_code_get_name(ev->type, ev->code),
			ev->value);
	}
	return 0;
}

static int
print_sync_event(struct input_event* ev)
{
	printf("SYNC: ");
	return print_event(ev);
}

void
JoysticThread(const char* file) {
	try {
		int fd = -1;
		struct libevdev* dev = nullptr;
		int rc;
		int reconnect = 1;
		bool localRunning = 1;
		joystickData localJoystick;

		while (localRunning) {

			if (!try_open_device(file, fd, dev)) {
				printf("Waiting for device...\n");
				reconnect = 1;
				std::this_thread::sleep_for(std::chrono::milliseconds(200));
				if (!running)
				{
					libevdev_free(dev);
					close(fd);
					dev = nullptr;
					fd = -1;
					localRunning = 0;
					{
						std::lock_guard<std::mutex> lock(joystickMutex);
						joystick = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
					}
				}
				continue;
			}

			while (localRunning) {
				std::this_thread::sleep_for(std::chrono::microseconds(1));
				struct input_event ev;
				rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL
					| LIBEVDEV_READ_FLAG_BLOCKING
					, &ev);

				if (reconnect)
				{
					reconnect = 0;
					printf("Input device ID: bus %#x vendor %#x product %#x\n",
						libevdev_get_id_bustype(dev),
						libevdev_get_id_vendor(dev),
						libevdev_get_id_product(dev));
					printf("Evdev version: %x\n", libevdev_get_driver_version(dev));
					printf("Input device name: \"%s\"\n", libevdev_get_name(dev));
					printf("Phys location: %s\n", libevdev_get_phys(dev));
					printf("Uniq identifier: %s\n", libevdev_get_uniq(dev));
					print_bits(dev);
					print_props(dev);
				}


				if (rc == LIBEVDEV_READ_STATUS_SUCCESS) {
					parseEvent(&ev, &localJoystick);
					generateChecksum(localJoystick);

					{
						std::lock_guard<std::mutex> lock(joystickMutex);
						joystick = localJoystick;
					}

					printJoystickData(&localJoystick);
				}
				else if (rc == LIBEVDEV_READ_STATUS_SYNC) {
					while (rc == LIBEVDEV_READ_STATUS_SYNC) {
						print_sync_event(&ev);
						rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_SYNC, &ev);
					}
				}
				else if (rc == -ENODEV) {
					fprintf(stderr, "[DISCONNECTED] Device unplugged.\n");
					{
						std::lock_guard<std::mutex> lock(joystickMutex);
						joystick = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
					}
					break; // keluar dari loop pembacaan, masuk ke loop reconnect
				}
				else if (rc != -EAGAIN) {
					fprintf(stderr, "Error: %s\n", strerror(-rc));
					{
						std::lock_guard<std::mutex> lock(joystickMutex);
						joystick = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
					}
					//std::this_thread::sleep_for(std::chrono::milliseconds(1));
				}

				if (!running)
				{
					break;
				}
			}



			libevdev_free(dev);
			close(fd);
			dev = nullptr;
			fd = -1;
			if (!running)
			{
				localRunning = 0;
				{
					std::lock_guard<std::mutex> lock(joystickMutex);
					joystick = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
				}
				printf("Joystick thread exited.\n");
			}
			sleep(1); // tunggu sebelum mencoba lagi
		}
	}
	catch (const std::exception& e) {
		fprintf(stderr, "[JoystickThread Exception] %s\n", e.what());
	}
}

int
main(int argc, char** argv)
{
	signal(SIGINT, handle_sigint);  // Tangani Ctrl+C
	signal(SIGKILL, handle_sigint);  // Tangani Ctrl+C

	// Joystick Global Variables
	const char* file;
	const char* port;
	const char* serverIP;

	if (argc < 2) {
		fprintf(stderr, "Usage: %s /dev/input/eventX\n", argv[0]);
		return 1;
	}

	file = argv[1];
	port = (argc > 2) ? argv[2] : PORT; // Port default jika tidak diberikan
	serverIP = (argc > 3) ? argv[3] : SERVER_IP; // Port default jika tidak diberikan

	// Setup UDP thread
	std::thread t1(UDPthread, port, serverIP);
	std::thread t2(JoysticThread, file);

	// Wait for the UDP thread to start
	t1.join();
	t2.join();


	return 0;
}
