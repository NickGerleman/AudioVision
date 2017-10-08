#include "pch.h"
#include "AudioCloudSource.h"
#include "AudioPlayer.h"
#include "IMU.h"
#include "NavAudioManager.h"

int main(int argc, char** argv)
{
	using namespace std::chrono_literals;

	try
	{
		auto& cloudSource = AudioCloudSource::get();
		std::cout << "Hello World" << std::endl;
		NavAudioManager navManager;

		while (true)
		{
			Eigen::Matrix3f orientationMat;
			if (!IMU::get().getAngleBlocking(orientationMat, 30ms))
				continue;

			auto lookVector = orientationMat * Eigen::Vector3f(0, 0, -1);
			auto upVector = orientationMat * Eigen::Vector3f(0, 1, 0);
			float alTransform[6] = {
				lookVector.x(), lookVector.y(), lookVector.z(),
				upVector.x(), upVector.y(), upVector.z()
			};
			AudioPlayer::instance().SetListenerAtUp(alTransform);
			navManager.FadeAudioFrames();

			if (AudioCloudSource::get().hasNewData())
			{
				auto audioCloud = AudioCloudSource::get().copyLatestCloud();
				navManager.AddAudioFrame(audioCloud.spAudioPoints, audioCloud.timestamp);
			}
		}
	}
	catch (const std::exception& ex)
	{
		std::cout << ex.what() << std::endl;
	}

	return EXIT_SUCCESS;
}
