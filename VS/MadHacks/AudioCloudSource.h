#pragma once

struct AudioCloudRecord
{
	AudioCloudRecord() = default;

	AudioCloudRecord(const boost::shared_ptr<PointCloud>& spAudioPoints, const Timestamp& timestamp)
		: spAudioPoints(spAudioPoints)
		, timestamp(timestamp) {}

	boost::shared_ptr<const PointCloud> spAudioPoints;
	Timestamp timestamp;
};


class AudioCloudSource
{
public:
	AudioCloudSource(const AudioCloudSource&) = delete;

	static AudioCloudSource& get();
	bool hasNewData();
	AudioCloudRecord copyLatestCloud();

private:
	AudioCloudSource()
		: m_hasNewData(false) {}

	void startLoop();

	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> clusterCloud(const boost::shared_ptr<PointCloud>& spCloud);
	boost::shared_ptr<PointCloud> audioPointsFromClustered(const pcl::PointCloud<pcl::PointXYZRGB>& clusteredCloud);
	boost::shared_ptr<PointCloud> transformToWorld(const boost::shared_ptr<PointCloud>& spCloud, const Eigen::Matrix3f& currentRotation);

	bool m_hasNewData;
	AudioCloudRecord m_lastCloud;
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
	openni::Device m_cameraDevice;
	openni::VideoStream m_depthStream;
};

