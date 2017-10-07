#include "pch.h"
#include "AudioCloudSource.h"

int main(int argc, char** argv)
{
	using namespace std::chrono_literals;

	auto& cloudSource = AudioCloudSource::get();
	auto buffer = cloudSource.copyBuffer();

	std::cout << "Hello World" << std::endl;

	while (true)
	{
		std::this_thread::sleep_for(60s);
	}

	return EXIT_SUCCESS;
}
