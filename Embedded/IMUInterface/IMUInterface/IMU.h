#pragma once

#include "PeriodicTimer.h"

class IMU {
public:
	static IMU& get();
	~IMU();

	Eigen::Matrix3f getAngle() const;
	bool getAngleBlocking(Eigen::Matrix3f& out,
		const std::chrono::microseconds& maxDelay);

	bool isConnected() const;

private:
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

	const int BAUD_RATE = 115'200;
	static const int BUFFER_SIZE = 256;
	const int START_BYTE = 0x00;

	IMU(int comPort);

	void configComPort();

	void startReading();
	void cbRead(int bytesRead);
	bool parseAngle(std::vector<unsigned char>& buffer, Euler&);
	static Eigen::Matrix3f eulerToMatrix(const Euler& angle);

	std::string port;
	boost::asio::io_service ioService;
	std::unique_ptr<boost::asio::io_service::work> ioWork;
	boost::asio::serial_port comPort;

	std::array<unsigned char, BUFFER_SIZE> readBuffer;
	std::vector<unsigned char> partialPacket;

	bool connected;
	PeriodicTimer connectTimer;
	
	std::thread ioThread;

	Eigen::Matrix3f angle;
	mutable std::mutex angleMutex;
	mutable std::condition_variable angleCondition;
	mutable bool angleConditionBool, freshAngle;
};
