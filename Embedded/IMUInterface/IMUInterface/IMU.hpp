#pragma once

#include <thread>
#include <memory>
#include <array>
#include <vector>
#include <mutex>
#include <condition_variable>

#include <boost/asio.hpp>

#include "PeriodicTimer.hpp"

class IMU {
public:
#pragma pack(push)
#pragma pack(1)
	struct Euler {
		Euler();
		Euler(float, float, float);

		operator bool() const;

		std::string toString() const;

		float pitch, yaw, roll;
	};
#pragma pack(pop)

	IMU(int comPort);
	~IMU();

	Euler getAngle() const;
	bool getAngleBlocking(Euler& out,
		const std::chrono::microseconds& maxDelay);

	bool isConnected() const;

private:
	const int BAUD_RATE = 57600;
	static const int BUFFER_SIZE = 256;
	const int START_BYTE = 0x00;

	void configComPort();

	void startReading();
	void cbRead(int bytesRead);
	bool parseAngle(std::vector<unsigned char>& buffer, Euler&);

	std::string port;
	boost::asio::io_service ioService;
	std::unique_ptr<boost::asio::io_service::work> ioWork;
	boost::asio::serial_port comPort;

	std::array<unsigned char, BUFFER_SIZE> readBuffer;
	std::vector<unsigned char> partialPacket;

	bool connected;
	PeriodicTimer connectTimer;
	
	std::thread ioThread;

	Euler angle;
	mutable std::mutex angleMutex;
};