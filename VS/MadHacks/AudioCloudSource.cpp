#include "pch.h"
#include "AudioCloudSource.h"

const int BUFFER_SIZE = 50;

using namespace std::chrono;

AudioCloudSource& AudioCloudSource::get()
{
	static AudioCloudSource instance;
	instance.startLoop();
	return instance;
}


std::deque<AudioCloudRecord> AudioCloudSource::copyBuffer()
{
	std::lock_guard<std::mutex> recordLock(m_recordMutex);
	return m_records;
}


void AudioCloudSource::startLoop()
{
	m_CameraThread = std::thread([this]()
	{
		Camera depthCam;

		while (true)
		{
			Timestamp now = high_resolution_clock::now();
			auto spCamCloud = depthCam.captureFrame();
			auto spSegmentedPoints = segmentCloud(spCamCloud);
			auto spWorldAudioCloud = transformToWorld(spSegmentedPoints, Eigen::Matrix3f()); // Grab rotation data from Eric for here

			// block for scope
			{
				std::lock_guard<std::mutex> recordLock(m_recordMutex);
				m_records.emplace_front(spWorldAudioCloud, now );

				if (m_records.size() > BUFFER_SIZE)
					m_records.pop_back();
			}
		}
	});
}


boost::shared_ptr<PointCloud> AudioCloudSource::segmentCloud(const boost::shared_ptr<PointCloud>& spCloud)
{
	// TODO
	return boost::make_shared<PointCloud>();
}


boost::shared_ptr<PointCloud> AudioCloudSource::transformToWorld(const boost::shared_ptr<PointCloud>& spCloud, const Eigen::Matrix3f& currentRotation)
{
	// TODO
	return boost::make_shared<PointCloud>();
}


Camera::Camera()
{
	// TODO
}


boost::shared_ptr<PointCloud> Camera::captureFrame()
{
	// TODO
	return boost::make_shared<PointCloud>();
}


