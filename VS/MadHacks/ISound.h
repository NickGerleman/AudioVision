#pragma once

class ISound
{
public:
	virtual void SetGain(float gain) = 0;
	virtual void SetPos(float x, float y, float z) = 0;
	virtual void SetPitch(float pitch) = 0;
};

//using SoundList = std::shared_ptr<std::vector<std::shared_ptr<ISound>>>;

// request sound objects
// const SoundList& spSounds = spAudioPlayer->RequestSounds(spAudioPoints->size());