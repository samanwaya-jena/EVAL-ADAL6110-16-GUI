

double maxVelocity(30); // meters per second
double frameRate(100.0); // frames per second
double posDamping(0.6);  // damping to compute the last position of the track. Range 0-1.
double speedDamping(0.6)	// damping to compute the last velocity of the track
double trackTimeOut(.5)		// Track inactivity time out (seconds) 
double distanceThreshold(0.5) // Maximum error to correlate, in meters
double velocityErrorMax(2)   // Maximum velocity error to create new track (in meters per second)
double minimumDistance(1.0) // Minimum distance at which we consider an obstacle valid.


std::Vector<Track::Ptr> tracks; 
typedef std::Vector<Track::Ptr> TrackList;





void AcquisitionSequence::BuildTracks() 
{
	int tracksAdded = 0;

	int frameQty = acquisitionSequence->sensorFrames.size();
	uint32_t frameID =  acquisitionSequence->GetLastFrameID();
	float timeStamp = frameID / frameRate; 

	do // while tracksAdded
	{
		tracksAdded = 0;
		// For each detection- 
		// We build estimates on 3 frame history, so don't go to the bottom 
		for (int frameIndex = 0; frameIndex < frameQty-1; frameNo++) 
		{
			//
			//For each of the channels (to index the detections)
			for (int channelIndex = 0; channelIndex < channelQty; channelIndex++)
			{
				// For each of the detections in the current frame
				for (int detectionIndex = 0; detectionIndex < detections; detectionIndex++)
				{
					detection = acquisitionSequence->GetDetection(frameIndex, channelIndex, detectionIndex);

					// Negative or near zero detections are ignored.
					bool bDetectionValid = (detection.distance >= minimumDistance);
					if (bDetectionValid) 
					{
						trackAdded += FitDetectionToTrack(detection);	
					} // if (bDetectionValid)
				} // for detection index
			}// for channelIndex
		} // for (frameIndex)

	} while (tracksAdded);

	// Merge duplicate tracks and kill old tracks
	CleanTracks();
}

	

void AcquisitionSequence::CleanTracks()

{
	int trackQty = tracks.size();
	std::iterator<Track::Ptr, *Track::Ptr >  trackIterator = tracks.begin();

	while (trackIterator != tracks.end()) 
	{
		track = trackIterator.tracks[trackIndex];
		if ((timeStamp - track->timeStamp) > trackTimeOut) 
		{
			trackIterator = tracks.erase(trackIterator);
		}

		// Tracks that stay are cleaned for the next iteration
		else
		{
			track->detections.clear();
		}
	}// while..
}

int AcquisitionSequence::FitDetectionToTrack(Detection::Ptr & detection)
{
	// Scan to see if the detection belongs to an existing track
	int tracksAdded = 0;
	trackQty = tracks.size();
	for (int trackIndex = 0; trackIndex < trackQty) 
	{
		track = tracks[trackIndex];
		if (!track.contains(detection)) 
		{
			distanceDelta = track.distance - detection.distance;
			speedDelta = track.speed - detection.speed;
			if (distanceDelta <= distanceThreshold) 
			{
				// Ponderate the track speed and distance based 
				// on the additionnal track and estimated position.
				track->speed =  ((detection->distance - track->distance) / time) +
					((1-speedDamping) * track->speed);

				track->position = (detection->distance * positionDamping) +
					((1 - positionDamping) * (track->distance + (track->velocity * time));
				track->timeStamp = timeStamp;

				track.detections.push_back(detection);

				// Update detection information
				if (track->threatLevel > detection->threatLevel) 
				{
					detection->velocity = track->speed;
					detection->threatLevel = track->threatLevel;
					detection->trackID = track->trackID;
				}
			}
		} // for (tracklndex)

		if (detection->tracks.size() == 0) 
		{
			uint32_t frameID =  acquisitionSequence->GetLastFrameID());
			Track::Ptr track = Track::Ptr(new Track(trackIDGenerator++, distance, 0, frameID, frameID, Detection::eThreatNone));
			track->speed = 0;
			track->distance = detection->distance;
			track->timeStamp = timeStamp;
			track->firstTimeStamp = timeStamp;
			tracks.push_back(newTrack);
			track.detections.push_back(detection);
			tracksAdded++;

			detection->velocity = track->speed;
			detection->threatLevel = track->threatLevel;
			detection->trackID = track->trackID;
		}

	}
	return(tracksAdded);
}