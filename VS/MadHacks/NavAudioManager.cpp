#include "pch.h"
#include "NavAudioManager.h"
using namespace std::literals::chrono_literals;

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

void NavAudioManager::AddAudioFrame(const boost::shared_ptr<PointCloud>& spAudioPoints, const Timestamp& timetamp)
{
	// request sound objects
	const boost::shared_ptr<std::vector<ISound>>& spSounds = spAudioPlayer->RequestSounds(spAudioPoints->size());
	
	// set up each of the audio points
	for (int i = 0; i < spAudioPoints->size(); i++)
	{
		pcl::PointXYZ = (*spAudioPoints)[i];
		(*spSounds)[i].SetPos((*spAudioPoints)[i].x, (*spAudioPoints)[i].y, (*spAudioPoints)[i].z);
		(*spSounds)[i].SetGain(MAX_GAIN);
		(*spSounds)[i].SetPitch(CalcPitch();



			virtual void SetGain(float gain) = 0;
		virtual void SetPitch(float pitch) = 0;
	}
}

void NavAudioManager::FadeAudioFrames()
{

}

