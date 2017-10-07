#include "pch.h"
#include "AudioPlayer.h"


AudioPlayer::AudioPlayer()
{
	//open default audio device
	device = alcOpenDevice(NULL);

	

	//reset error state
	alGetError();

	alGenBuffers(num_buffers, buffer_names);
	ALenum error_code = alGetError();
	if (error_code != AL_NO_ERROR) {
		throw "AudioPlayer was unable to generate buffers";
	}



}

AudioPlayer::~AudioPlayer()
{
	alcCloseDevice(device);
	alDeleteBuffers(num_buffers, buffer_names);
}
