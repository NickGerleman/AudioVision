#include "IMU.hpp"

#include <iostream>
#include <exception>

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
	, connectTimer{ ioService, 500ms, [this]() {
			std::cout << "[Info] IMU::connectTimer: Tick" << std::endl;
			
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
	, ioThread{ [this]() { ioService.run(); } } {

	static_assert(sizeof(Euler) == 12, "sizeof(Euler) must be 12");

	//Start attempting to connect
	connectTimer.start();
}

IMU::~IMU() {
	ioWork.reset();
	ioThread.join();
}

IMU::Euler IMU::getAngle() const {
	std::unique_lock<std::mutex> angleLock(angleMutex);
	return angle;
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
	}
	catch (const std::exception& e) {
		std::cerr << "[Error] IMU::IMU: " << e.what() << std::endl;

		connected = false;
		connectTimer.start();
	}
}

void IMU::startReading() {
	async_read(comPort, buffer(readBuffer),
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
			connectTimer.start();
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
			angle = newAngle;
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