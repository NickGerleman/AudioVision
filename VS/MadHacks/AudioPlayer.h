#pragma once


class AudioPlayer {
public:
	AudioPlayer();
	~AudioPlayer();

	static constexpr const int num_buffers = 1;
private:
	ALCdevice* device;
	ALuint buffer_names[num_buffers];
};