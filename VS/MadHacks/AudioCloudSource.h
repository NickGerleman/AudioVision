#pragma once

struct AudioCloudRecord
{
	AudioCloudRecord(const boost::shared_ptr<PointCloud>& spAudioPoints, const Timestamp& timetamp)
		: spAudioPoints(spAudioPoints)
		, timestamp(timestamp) {}

	const boost::shared_ptr<const PointCloud> spAudioPoints;
	const Timestamp timestamp;
};


class AudioCloudSource
{
public:
	static AudioCloudSource& get();
	std::deque<AudioCloudRecord> copyBuffer();

private:
	AudioCloudSource() = default;
	void startLoop();

	boost::shared_ptr<PointCloud> segmentCloud(const boost::shared_ptr<PointCloud>& spCloud);
	boost::shared_ptr<PointCloud> transformToWorld(const boost::shared_ptr<PointCloud>& spCloud, const Eigen::Matrix3f& currentRotation);

	std::deque<AudioCloudRecord> m_records;
	std::mutex m_recordMutex;
	std::thread m_CameraThread;
};


class Camera
{
	friend AudioCloudSource;

public:
	// Capture a frame within the current coordinate system
	boost::shared_ptr<PointCloud> captureFrame();

private:
	Camera();
};

