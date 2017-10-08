#include "pch.h"

#include "IMU.h"

#include <chrono>

using namespace std::chrono_literals;

int main() {
	std::cout << "[Info] Starting IMU..." << std::flush;
	auto& imu = IMU::get();
	std::cout << "done" << std::endl;

	while (true) {
		static int count = 0;

		Eigen::Matrix3f angle;
		if (!imu.getAngleBlocking(angle, 20ms)) {
			std::cout << "[Info] Angle Blocking Timeout"
				<< std::endl;
		}
		else {
			count++;
			if (count / 10) {

			}
			std::cout << count << std::endl;
		}
	}

	return 0;
}