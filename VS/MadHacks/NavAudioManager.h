#pragma once
#include "ISound.h"
#include "AudioPlayer.h"

struct AudioFrame
{
	AudioFrame() = default;

	AudioFrame(const SoundList spSounds, const Timestamp& timetamp)
		: spSounds(spSounds)
		, timestamp(timestamp) {}

	SoundList spSounds;
	Timestamp timestamp;
};

class NavAudioManager
{
public:
	NavAudioManager(const std::shared_ptr<AudioPlayer>& spAudioPlayer)
		: spAudioPlayer(spAudioPlayer) {}

	NavAudioManager(const NavAudioManager&) = delete;

	void AddAudioFrame(const boost::shared_ptr<PointCloud>& spAudioPoints, const Timestamp& timetamp);
	void FadeAudioFrames();

private:


	std::shared_ptr<std::vector<AudioFrame>> spFrames;
	std::shared_ptr<AudioPlayer> spAudioPlayer;
};