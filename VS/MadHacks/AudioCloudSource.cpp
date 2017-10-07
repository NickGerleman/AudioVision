#include "pch.h"
#include "AudioCloudSource.h"

const int BUFFER_SIZE = 50;

using namespace std::chrono;
using namespace openni;

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
		pcl::visualization::PCLVisualizer viz;
		viz.addPointCloud(boost::make_shared<PointCloud>());

		while (true)
		{
			Timestamp now = high_resolution_clock::now();
			auto spCamCloud = depthCam.captureFrame();
			auto spSegmentedPoints = segmentCloud(spCamCloud);

			viz.updatePointCloud(spCamCloud);
			viz.spinOnce();
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
	m_CameraThread.detach();
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
	static bool initialized = false;
	if (initialized)
		std::cerr << "Can't double construct camera" << std::endl;

	if (OpenNI::initialize() != STATUS_OK)
		std::cerr << "Can't initialize. Please debug me:" << OpenNI::getExtendedError() << std::endl;
	
	if (m_cameraDevice.open(ANY_DEVICE) != STATUS_OK)
		std::cerr << "Can't open device. Please debug me:" << OpenNI::getExtendedError() << std::endl;

	auto* pDepthInfo = m_cameraDevice.getSensorInfo(SENSOR_DEPTH);
	auto& supportedModes = pDepthInfo->getSupportedVideoModes();

	if (m_depthStream.create(m_cameraDevice, SENSOR_DEPTH) != STATUS_OK)
		std::cerr << "Can't open depth stream. Please debug me:" << OpenNI::getExtendedError() << std::endl;

	VideoMode vgaMode;
	vgaMode.setFps(30);
	vgaMode.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
	vgaMode.setResolution(640, 480);
	if (m_depthStream.setVideoMode(vgaMode) != STATUS_OK)
		std::cerr << "Can't set mode. Please debug me:" << OpenNI::getExtendedError() << std::endl;

	if (m_depthStream.start())
		std::cerr << "Can't start depth stream. Please debug me:" << OpenNI::getExtendedError() << std::endl;

	initialized = true;
}


boost::shared_ptr<PointCloud> Camera::captureFrame()
{
	VideoFrameRef frame;
	if (m_depthStream.readFrame(&frame) != STATUS_OK)
		std::cerr << "Can't read frame. Please debug me:" << OpenNI::getExtendedError() << std::endl;

	auto spWorldCloud =  boost::make_shared<PointCloud>();
	
	const auto* frameData = static_cast<const DepthPixel*>(frame.getData());
	for (int y = 0; y < frame.getHeight(); y++)
	{
		const DepthPixel* rowPtr = frameData + (y * frame.getWidth());
		for (int x = 0; x < frame.getWidth(); x++)
		{
			DepthPixel cameraDepth = rowPtr[x];
			if (cameraDepth == 0)
				continue;

			pcl::PointXYZ worldPt;
			CoordinateConverter::convertDepthToWorld(m_depthStream, x, y, cameraDepth, &worldPt.x, &worldPt.y, &worldPt.z);
			
			// change coordinate system
			worldPt.z *= -1;
			worldPt.x *= -1;

			spWorldCloud->push_back(worldPt);
		}	
	}

	return spWorldCloud;
}


