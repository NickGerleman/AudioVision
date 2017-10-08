#pragma once

#pragma pack(push)
#pragma pack(1)
typedef struct {
	uint32_t chunkId;
	uint32_t chunkSize;
	uint8_t unused[14];
	uint16_t numChannels;
	uint32_t sampleRate;
	uint32_t unusedb;
	uint16_t unusedc;
	uint16_t bitsPerSample;
	uint32_t unusedd;
	uint32_t dataChunkSize;

} wave_preamble_t;
#pragma pack(pop)

static_assert(sizeof(wave_preamble_t) == 44, "Wave_preamble not packing correctly");

class AudioPlayer {
public:
	AudioPlayer();
	~AudioPlayer();

	static constexpr const int numBuffers = 1;
	static constexpr const int maxNumSoundSources = 100;
private:
	/*
		Loads the pcm data from the specified wave file, caller expected to free pointer when done with the data.
	*/
	void AudioPlayer::loadWave(const char* filename, ALuint buffer);
	ALenum getOpenAlSoundFormat(uint16_t numChannels, uint8_t bitsPerSample);
	ALCdevice* device;
	ALuint bufferNames[numBuffers];
	ALuint sourceNames[maxNumSoundSources];
	std::vector<ALvoid*> pcms;
	ALCcontext* context;

};