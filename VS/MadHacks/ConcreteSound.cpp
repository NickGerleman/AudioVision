#include "pch.h"
#include "ConcreteSound.h"

ConcreteSound::ConcreteSound(ALuint sourceName, std::shared_ptr<AudioPlayer> player) : sourceName(sourceName), player(player)
{
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


