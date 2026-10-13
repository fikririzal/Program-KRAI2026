#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>

//#include <CppLinuxSerial/SerialPort.hpp>
//using namespace mn::CppLinuxSerial;

// Multi Threading
//#include <iostream>
#include <thread>
#include <mutex>
#include <csignal>
#include <chrono>
#include <atomic>

// Serial Libs
//#include <iostream>
#include <fcntl.h>
#include <termios.h>
//#include <unistd.h>
//#include <cstring>

#pragma pack(push, 1)
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
#pragma pack(pop)

joystickData joy;
// Mutex for thread safety
std::mutex joyMutex;
std::atomic<bool> running(true); // digunakan oleh semua thread

#define PORT "1412"
#define REMOTE_IP "192.25.2.2"
#define INTERFACE_IP "192.25.2.1"
#define BUFFER_SIZE 128

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

bool verifyChecksum(const joystickData& joy) {
	const uint8_t* start = reinterpret_cast<const uint8_t*>(&joy.LX);
	const size_t length = offsetof(joystickData, SUM) - offsetof(joystickData, LX);

	return crc8(start, length) == joy.SUM;
}

static void
printJoystickData(const joystickData* js) {
	printf("LX:%4d LY:%4d RX:%4d RY:%4d THR:%3u BRK:%3u BTN:%4u MISC:%1u DPAD:%1u BAT:%3u SUM:%3u INC:%3u\n",
		js->LX, js->LY, js->RX, js->RY,
		js->THR, js->BRK,
		js->BTN, js->MISC, js->DPAD,
		js->BAT, js->SUM,js->INC);
}

void handle_sigint(int signum) {
	running = false;
	printf("Caught signal %d, exiting...\n", signum);
}

void UDPthread(const char* port, const char* interfaceIP) {
	try {
		int sockfd;
		char buffer[BUFFER_SIZE];
		struct sockaddr_in servaddr {}, cliaddr{};
		socklen_t len = sizeof(cliaddr);
		joystickData localData = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

		// 1. Buat socket
		sockfd = socket(AF_INET, SOCK_DGRAM, 0);
		if (sockfd < 0) {
			perror("Socket creation failed");
			return;
		}

		int flags = fcntl(sockfd, F_GETFL, 0);
		fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

		// 2. Siapkan alamat server
		servaddr.sin_family = AF_INET;          // IPv4
		servaddr.sin_addr.s_addr = inet_addr(interfaceIP);  // Terima dari interface
		uint16_t _port = static_cast<uint16_t>(std::stoul(port));
		servaddr.sin_port = htons(_port);

		//// 3. Bind socket ke alamat dan port
		//if (bind(sockfd, (const struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
		//	perror("Bind failed");
		//	close(sockfd);
		//	return;
		//}

		int tryBind = 1;
		while (tryBind) {
			if (!running)
			{
				tryBind = 0;
			}
			if (bind(sockfd, (const struct sockaddr*)&servaddr, sizeof(servaddr)) == 0) {
				printf("Bind berhasil pada interface %s, port %d\n",
					inet_ntoa(servaddr.sin_addr),
					ntohs(servaddr.sin_port));
				tryBind = 0;
				break;
			}

			if (errno == EADDRNOTAVAIL) {
				fprintf(stderr, "Bind gagal: Alamat belum tersedia. Mencoba lagi dalam 200ms...\n");
			}
			else {
				perror("Bind gagal");
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Delay sebelum mencoba lagi
		}

		std::cout << "UDP server listening on port " << port << "...\n";

		// 4. Terima data
		while (running) {
			ssize_t n = recvfrom(sockfd, buffer, BUFFER_SIZE, 0,
				(struct sockaddr*)&cliaddr, &len);

			if (n < 0) {
				if (errno == EWOULDBLOCK || errno == EAGAIN) {
					// Tidak ada data sekarang, retry
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
					continue;
				}
				perror("recvfrom error");
				break;
			}

			// Tampilkan data sebagai hex
			/*std::cout << "Received " << n << " bytes from "
				<< inet_ntoa(cliaddr.sin_addr) << ":" << ntohs(cliaddr.sin_port)
				<< " : ";
			for (ssize_t i = 0; i < n; ++i)
				printf("%02X ", static_cast<unsigned char>(buffer[i]));
			std::cout << std::endl;*/

			// Pastikan ukuran sesuai:
			if (sizeof(joystickData) <= sizeof(buffer)) {
				std::memcpy(&localData, buffer, sizeof(joystickData));
				
				// Sekarang struct `localData` sudah terisi
				if (verifyChecksum(localData))
				{
					//std::cerr << "Data correct" << std::endl;
					{
						std::lock_guard<std::mutex> lock(joyMutex);
						joy = localData;
					}
					printJoystickData(&localData);
				}
				else {
					std::cerr << "Invalid checksum\n";
				}
			}
			else {
				std::cerr << "Data too small!" << std::endl;
			}
		}

		close(sockfd);
		std::cout << "UDP Closed" << std::endl;

	}
	catch (const std::exception& e) {
		fprintf(stderr, "[UDPthread Exception] %s\n", e.what());
	}
}

void SerialThread(const char* file) {
	try {
		//SerialPort Serial(file, BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
		//Serial.SetTimeout(0);
		//Serial.Open();
		//Serial.Write("Hello, World!!");
		//tcdrain()

		//Serial.Write("Hello, World!!\n");

		//const char* portname = "/dev/ttyUSB0";  // Ganti dengan port yang sesuai

		/*int fd = open(file, O_RDWR | O_NOCTTY | O_SYNC);
		if (fd < 0) {
			perror("open");
			return;
		}*/
setupSerial:
		int fd = 0;
		int tryConnect = 1;
		while (tryConnect) {
			fd = 0;
			if (!running)
			{
				tryConnect = 0;
			}
			fd = open(file, O_RDWR | O_NOCTTY | O_SYNC);
			if (fd >= 0) {
				printf("Berhasil membuka %s\n", file);
				tryConnect = 0;
				break;
			}

			perror("Gagal membuka device, mencoba lagi dalam 200 milidetik...");
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}

		struct termios tty;
		memset(&tty, 0, sizeof tty);
		if (tcgetattr(fd, &tty) != 0) {
			perror("tcgetattr");
			close(fd);
			return;
		}

		// Set baudrate 115200
		cfsetospeed(&tty, B115200);
		cfsetispeed(&tty, B115200);

		// 8 data bit
		tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;

		// No parity
		tty.c_cflag &= ~PARENB;

		// One stop bit
		tty.c_cflag &= ~CSTOPB;

		// No flow control
		tty.c_cflag &= ~CRTSCTS;

		// Local mode & enable receiver
		tty.c_cflag |= (CLOCAL | CREAD);

		// Raw input/output
		tty.c_lflag = 0;
		tty.c_iflag &= ~(IXON | IXOFF | IXANY);
		tty.c_oflag = 0;

		// Non-blocking read settings
		tty.c_cc[VMIN] = 0;
		tty.c_cc[VTIME] = 5;  // 0.5 second read timeout

		joystickData localData;

		if (tcsetattr(fd, TCSANOW, &tty) != 0) {
			perror("tcsetattr");
			close(fd);
			return;
		}

		while (running) {
			//const char* msg = "Data ke serial 115200 8N1\n";
			static uint8_t lastINC = 0;
			static auto lastChange = std::chrono::steady_clock::now();

			{
				std::lock_guard<std::mutex> lock(joyMutex);
				localData = joy;
			}

			// Cek perubahan INC
			if (localData.INC != lastINC) {
				lastINC = localData.INC;
				lastChange = std::chrono::steady_clock::now();
			}
			else {
				auto now = std::chrono::steady_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastChange).count();
				if (duration >= 200) {
					memset(&localData, 0, sizeof(localData));
				}
			}

			// Kirim struct secara langsung sebagai binary
			localData.identifier = 0xcafe;
			ssize_t bytes = write(fd, &localData, sizeof(joystickData) - 1);// data INC tidak dikirim
			if (bytes < 0) {
				perror("write failed");
				close(fd);
				goto setupSerial;
			}

			tcdrain(fd);  // Tunggu sampai semua data terkirim - Sudah Bloking
			std::this_thread::sleep_for(std::chrono::milliseconds(1)); // interval pengiriman
		}

		close(fd);
		std::cout << "Serial Closed" << std::endl;
	}
	catch (const std::exception& e) {
		fprintf(stderr, "[SerialThread Exception] %s\n", e.what());
	}
}

int main(int argc, char** argv) {
	signal(SIGINT, handle_sigint);  // Tangani Ctrl+C
	signal(SIGKILL, handle_sigint);  // Tangani Ctrl+C

	const char* file;
	const char* port;
	const char* interfaceIP;
	if (argc < 3) {
		fprintf(stderr, "Usage: %s <serial_device>\n", argv[0]);
		return 1;
	}
	file = argv[1];
	port = (argc > 2) ? argv[2] : PORT; // Port default jika tidak diberikan
	interfaceIP = (argc > 3) ? argv[3] : INTERFACE_IP;

	std::thread t1(UDPthread, port, interfaceIP);
	std::thread t2(SerialThread, file);

	t1.join();
	t2.join();

	return 0;
}
