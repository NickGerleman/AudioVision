#pragma once

class ISound
{
	ISound() {};
	virtual ~ISound() {};
	virtual void SetGain(float gain) = 0;
	virtual void SetPos(float x, float y, float z) = 0;
	virtual void SetPitch(float pitch) = 0;
};

// const boost::shared_ptr<std::vector<ISound>>& spSounds = spAudioPlayer->RequestSounds(spAudioPoints->size());