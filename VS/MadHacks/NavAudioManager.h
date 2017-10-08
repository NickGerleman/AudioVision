#pragma once
#include "ISound.h"
#include "AudioPlayer.h"

struct AudioFrame
{
	AudioFrame() = default;

	AudioFrame(std::shared_ptr<std::vector<std::shared_ptr<ISound>>> spSounds, const Timestamp& timestamp)
		: spSounds(spSounds)
		, timestamp(timestamp) {}

	std::shared_ptr<std::vector<std::shared_ptr<ISound>>> spSounds;
	Timestamp timestamp;
};

class NavAudioManager
{
public:
	NavAudioManager()
		: audioPlayer(AudioPlayer::instance())
		, spFrames(std::make_shared<std::vector<AudioFrame>>()) {}

	NavAudioManager(const NavAudioManager&) = delete;

	void AddAudioFrame(const boost::shared_ptr<const PointCloud>& spAudioPoints, const Timestamp& timetamp);
	void FadeAudioFrames();

private:


	std::shared_ptr<std::vector<AudioFrame>> spFrames;
	AudioPlayer& audioPlayer;
};