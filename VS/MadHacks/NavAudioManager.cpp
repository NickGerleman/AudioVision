#include "pch.h"
#include "NavAudioManager.h"
using namespace std::literals::chrono_literals;
using namespace std::chrono;

const float MIN_GAIN = 0.0f;
const float MAX_GAIN = 1.0f;
const float MIN_PITCH = 0.1f;
const float MAX_PITCH = 2.0f;
const float MIN_DIST = 0.0f;
const float MAX_DIST = 10000.0f;
const std::chrono::microseconds FADE_TIME_US = 10ms;

float CalcPitch(float dist)
{
	// clamp
	dist = std::fabs(dist);
	dist = std::fmax(dist, MIN_DIST);
	dist = std::fmin(dist, MAX_DIST);

	// interpolate
	float fraction = (1.0f - ((dist - MIN_DIST) / (MAX_DIST - MIN_DIST)));
	float pitch = MIN_PITCH + (fraction * (MAX_PITCH - MIN_PITCH));

	return pitch;
}

float CalcGain(std::chrono::microseconds timePast)
{
	// clamp
	timePast = std::min(timePast, FADE_TIME_US);
	double delta = static_cast<double>(timePast.count());

	// interpolate
	float fraction = (1.0f - (delta / (FADE_TIME_US.count())));
	float gain = MIN_GAIN + (fraction * (MAX_GAIN - MIN_GAIN));

	return gain;
}

void NavAudioManager::AddAudioFrame(const boost::shared_ptr<PointCloud>& spAudioPoints, const Timestamp& timetamp)
{
	// check if there are enough sounds left
	while (spAudioPlayer->SoundsLeft() < spAudioPoints->size())
	{
		// recycle the oldest sounds
		spAudioPlayer->FreeSounds((*spFrames)[0].spSounds);
		
		// remove the oldest sound frame
		spFrames->erase(spFrames->begin());
	}

	// request sound objects
	std::shared_ptr<std::vector<std::shared_ptr<ISound>>> spSounds = spAudioPlayer->RequestSounds(spAudioPoints->size());
	
	// set up each of the audio points
	for (int i = 0; i < spAudioPoints->size(); i++)
	{
		pcl::PointXYZ& point = (*spAudioPoints)[i];
		std::shared_ptr<ISound>& sound = (*spSounds)[i];
		sound->SetPos(point.x, point.y, point.z);
		sound->SetPitch(CalcPitch(point.z));
		sound->SetGain(MAX_GAIN);
	}

	// make the audio frame
	AudioFrame frame = AudioFrame(spSounds, timetamp);

	// add the frame
	spFrames->push_back(frame);
}

void NavAudioManager::FadeAudioFrames()
{
	Timestamp captureTime = high_resolution_clock::now();

	// check if there are enough sounds left
	while ((std::chrono::duration_cast <std::chrono::microseconds> (captureTime - (*spFrames)[0].timestamp)) > FADE_TIME_US )
	{
		// recycle the oldest sounds
		spAudioPlayer->FreeSounds((*spFrames)[0].spSounds);

		// remove the oldest sound frame
		spFrames->erase(spFrames->begin());
	}

	// attenuate the rest of the sounds
	for( const auto& soundFrame : *spFrames )
	{
		for (const auto& sound : *soundFrame.spSounds)
		{
			std::chrono::microseconds timePast = std::chrono::duration_cast <std::chrono::microseconds> (captureTime - soundFrame.timestamp);
			sound->SetGain(CalcGain(timePast));
		}
	}
}

