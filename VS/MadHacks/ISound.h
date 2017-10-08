#pragma once

class ISound
{
public:
	virtual void SetGain(float gain) = 0;
	virtual void SetPos(float x, float y, float z) = 0;
	virtual void SetPitch(float pitch) = 0;
};
// request sound objects
// const SoundList& spSounds = spAudioPlayer->RequestSounds(spAudioPoints->size());