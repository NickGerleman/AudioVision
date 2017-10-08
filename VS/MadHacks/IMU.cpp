#include "pch.h"
#include "IMU.h"

using namespace std;
using namespace boost::asio;
using namespace std::chrono_literals;

IMU::Euler::Euler()
	: pitch{ 0.f }
	, yaw{ 0.f }
	, roll{ 0.f } {
}

IMU::Euler::Euler(float _pitch, float _yaw, float _roll)
	: pitch{ _pitch }
	, yaw{ _yaw }
	, roll{ _roll } {
}

IMU::Euler::operator bool() const {
	return (pitch <= 180.f && pitch >= -180.f)
		&& (yaw <= 180.f && yaw >= -180.f)
		&& (roll <= 180.f && roll >= -180.f);
}

std::string IMU::Euler::toString() const {
	return std::string("(") + std::to_string(pitch) + ", " +
		std::to_string(yaw) + ", " + std::to_string(roll) + ")";
}

IMU::IMU(int _comNumber)
	: ioWork{ std::make_unique<io_service::work>(ioService) }
	, port{ std::string("COM") + std::to_string(_comNumber) }
	, comPort{ ioService }
	, connected{ false }
	, connectTimer{ ioService, 5000ms, [this]() {
			try {
				comPort.open(port);
				if (comPort.is_open()) {
					connectTimer.stop();
					connected = true;

					std::cout << "[Info] IMU: Device connected" << std::endl;

					configComPort();
					startReading();
				}
			}
			catch (const std::exception& e) {
				std::cout << "[Error] IMU::connectTimer: " << e.what()
					<< std::endl;
			}
		} }
	, ioThread{ [this]() { ioService.run(); } }
	, angleConditionBool{ false }
	, freshAngle{ false } {

	static_assert(sizeof(Euler) == 12, "sizeof(Euler) must be 12");

	//Start attempting to connect
	connectTimer.start();
}

IMU& IMU::get()
{
	static IMU instance(3);
	return instance;
}

IMU::~IMU() {
	ioWork.reset();
	ioThread.join();
}

Eigen::Matrix3f IMU::getAngle() const {
	std::unique_lock<std::mutex> angleLock(angleMutex);

	return angle;
}

bool IMU::getAngleBlocking(Eigen::Matrix3f& out,
	const std::chrono::microseconds& maxDelay) {
	std::unique_lock<std::mutex> angleLock(angleMutex);
	
	if (freshAngle) {
		out = angle;
		freshAngle = false;

		return true;
	}
	else {
		angleConditionBool = false;

		if (angleCondition.wait_for(angleLock, maxDelay,
			[this]() { return angleConditionBool; })) {
			out = angle;

			return true;
		}
		else {
			if (angleConditionBool) {
				std::cout << "[Error] IMU::getAngleBlocking: angleConditionBool=true" << std::endl;
			}
			return false;
		}
	}
}

bool IMU::isConnected() const {
	return connected;
}

void IMU::configComPort() {
	try {
		comPort.set_option(serial_port_base::baud_rate(BAUD_RATE));
		comPort.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
		comPort.set_option(serial_port_base::parity(serial_port_base::parity::none));
		comPort.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
		comPort.set_option(serial_port_base::character_size(8));

		auto nativeHandle = comPort.native_handle();
		COMMTIMEOUTS timeouts;
		timeouts.ReadIntervalTimeout = MAXDWORD;
		timeouts.ReadTotalTimeoutMultiplier = 0;
		timeouts.ReadTotalTimeoutConstant = 0;
		timeouts.WriteTotalTimeoutMultiplier = 0;
		timeouts.WriteTotalTimeoutConstant = 0;
		SetCommTimeouts(nativeHandle, &timeouts);
	}
	catch (const std::exception& e) {
		std::cerr << "[Error] IMU::IMU: " << e.what() << std::endl;

		connected = false;
		connectTimer.start();
	}
}

void IMU::startReading() {
	async_read(comPort, buffer(readBuffer), transfer_at_least(1),
		[this](const boost::system::error_code& ec, size_t bytesTransferred) {
		if (ec) {
			if (ec) {
				std::cerr << "[Error] IMU::cbRead: " << ec.message() << std::endl;
			}
			std::cout << "[Info] IMU: Device disconnected" << std::endl;
			connected = false;
			try {
				comPort.close();
			}
			catch (const std::exception& e) {
				std::cerr << "[Error] IMU::cbRead: " << e.what() << std::endl;
			}
			connectTimer.start(false);
		}
		else {
			cbRead(bytesTransferred);
		}
	});
}

void IMU::cbRead(int bytesRead) {
	partialPacket.insert(partialPacket.end(), readBuffer.begin(),
		readBuffer.begin() + bytesRead);

	Euler newAngle;
	while (parseAngle(partialPacket, newAngle)) {
		if (newAngle) {
			//TODO: Consider queueing new angles
			std::unique_lock<std::mutex> angleLock(angleMutex);
			angle = eulerToMatrix(newAngle);
			freshAngle = true;
			angleConditionBool = true;
			angleCondition.notify_all();
		}
		else {
			std::cout << "[Error] IMU::cbRead: Invalid angle: "
				<< newAngle.toString() << std::endl;
		}
	}

	startReading();
}

bool IMU::parseAngle(std::vector<unsigned char>& buffer, Euler& out) {
	while (buffer.size() > 0) {
		if (buffer[0] != START_BYTE) {
			std::cout << "[Error] IMU::parseAngle: Invalid start byte ("
				<< (int)buffer[0] << ")" << std::endl;
			buffer.erase(buffer.begin());
		}
		else if (buffer.size() >= (1 + sizeof(out))) {
			std::memcpy(&out, buffer.data() + 1, sizeof(out));
			buffer.erase(buffer.begin(), buffer.begin() + sizeof(out) + 1);

			return true;
		}
		else {
			return false;
		}
	}
}

Eigen::Matrix3f IMU::eulerToMatrix(const Euler& angle) {
	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(angle.pitch, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(angle.yaw, Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(-angle.roll, Eigen::Vector3f::UnitZ());

	return m;
}