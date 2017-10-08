#include "pch.h"
#include "ConcreteSound.h"

ConcreteSound::ConcreteSound(ALuint sourceName, ALuint buffer) : sourceName(sourceName)
{
	alSourcei(sourceName, AL_BUFFER, buffer);
	alSourcei(sourceName, AL_LOOPING, AL_TRUE);
	alSourcef(sourceName, AL_REFERENCE_DISTANCE, 50);
	//alSourcePlay(sourceName);
}

void ConcreteSound::StopPlaying()
{
	ALint state;
	alGetSourcei(sourceName, AL_SOURCE_STATE, &state);

	if (state == AL_PLAYING) {
		alSourcePause(sourceName);
	}
}

void ConcreteSound::StartPlaying()
{
	ALint state;
	alGetSourcei(sourceName, AL_SOURCE_STATE, &state);

	if (state != AL_PLAYING) {
		alSourcePlay(sourceName);
	}

}


void ConcreteSound::SetGain(float gain)
{
	alSourcef(sourceName, AL_GAIN, gain);
}

void ConcreteSound::SetPos(float x, float y, float z)
{
	alSource3f(sourceName, AL_POSITION, x, y, z);
	StartPlaying();
}

void ConcreteSound::SetPitch(float pitch)
{
	alSourcef(sourceName, AL_PITCH, pitch);
}


