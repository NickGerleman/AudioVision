#include "pch.h"
#include "ConcreteSound.h"

ConcreteSound::ConcreteSound(ALuint sourceName, ALuint buffer) : sourceName(sourceName)
{
	alSourcei(sourceName, AL_BUFFER, buffer);
	alSourcei(sourceName, AL_LOOPING, AL_TRUE);
	SetGain(0);
}

void ConcreteSound::StopPlaying()
{
	ALint state;
	alGetSourcei(sourceName, AL_SOURCE_STATE, &state);

	if (state == AL_PLAYING) {
		alSourceStop(sourceName);
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
}

void ConcreteSound::SetPitch(float pitch)
{
	alSourcef(sourceName, AL_PITCH, pitch);
}


