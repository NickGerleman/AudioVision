#include <iostream>

#include "IMU.hpp"

int main() {
	std::cout << "[Info] Starting IMU..." << std::flush;
	IMU imu(3);
	std::cout << "done" << std::endl;

	while (true) {
		std::this_thread::sleep_for(std::chrono::seconds(10));
	}

	return 0;
}