#pragma once
#include "ISound.h"
#include "AudioPlayer.h"

class ConcreteSound : public ISound {
public:

	// Inherited via ISound
	virtual void SetGain(float gain) override;
	virtual void SetPos(float x, float y, float z) override;
	virtual void SetPitch(float pitch) override;
	ConcreteSound(ALuint sourceName, ALuint buffer);
	void StopPlaying();
	void StartPlaying();

private:
	const ALuint sourceName;
	const std::shared_ptr<AudioPlayer> player;
	


};