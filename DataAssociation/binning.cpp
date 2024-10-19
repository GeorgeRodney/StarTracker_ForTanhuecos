#include "TrackFileMgr.hpp"

void TrackFileMgr::binningAssociate()
{
    // DONT RUN IF THERE IS NOTHING TO DO
    if ((m_tracks.numTracks == 0) || (m_dets.numDets == 0))
    {
        return;
    }

    vector<int> state(STATE_MAX, -1); // Index is Det. Value is Track
    vector<vector<int>> tracksToDets(m_tracks.numTracks, vector<int>(m_dets.numDets, -1));
    vector<vector<int>> detsToTracks(m_dets.numDets, vector<int>(m_tracks.numTracks, -1));
    vector<int> trackHits(TRACK_MAX, 0);
    vector<int> detHits(DET_MAX, 0);
    int trkHitIdx = 0;
    int detHitIdx = 0;
    double cost;

    double minVal = 100000.0;
    int minValIdx = -1;

    // Gate the m_tracks
    for(int16_t trackIdx = 0; trackIdx < m_tracks.numTracks; trackIdx++)
    {
        for (int16_t detIdx = 0; detIdx < m_dets.numDets; detIdx++)
        {
            if (m_tracks.trackFiles[trackIdx].state != CLOSED)
            {
                // CALCULATE GATE
                double diff = statisticalDifferance(trackIdx, m_tracks, detIdx, m_dets);
                if (diff < m_tracks.trackFiles[trackIdx].gate)
                {
                    // Store track stuff
                    trackHits[trackIdx]++;
                    for (int trkHitIdx = 0; trkHitIdx < DET_MAX; trkHitIdx++)
                    {
                        if (tracksToDets[trackIdx][trkHitIdx] == -1)
                        {
                            tracksToDets[trackIdx][trkHitIdx] = detIdx;
                            break;
                        }
                    }
                    
                    // Store Det Info
                    detHits[detIdx]++;
                    for (int detHitIdx = 0; detHitIdx < TRACK_MAX; detHitIdx++)
                    {
                        if (detsToTracks[detIdx][detHitIdx] == -1)
                        {
                            detsToTracks[detIdx][detHitIdx] = trackIdx;
                            break;
                        }
                    }
                }

            }
        }
    }

    // ASSOCIATE
    for(int16_t trackIdx = 0; trackIdx < TRACK_MAX; trackIdx++)
    {   
        // Associate the m_tracks with one detection in their gate
        if(trackHits[trackIdx] == 1)
        {
            m_tracks.trackFiles[trackIdx].corrDet = tracksToDets[trackIdx][0];
            m_dets.detList[m_tracks.trackFiles[trackIdx].corrDet].correlated = true;
            m_dets.detList[m_tracks.trackFiles[trackIdx].corrDet].corrTrack = trackIdx;

            // Remove from lists
            trackHits[trackIdx] = 0;
            tracksToDets[trackIdx][0] = -1;

        }

        // Associate track with detection with the smallest cost
        if (trackHits[trackIdx] > 1)
        {
            for(int idx = 0; idx < trackHits[trackIdx]; idx++)
            {
                int detectionToTest = tracksToDets[trackIdx][idx];
                tracksToDets[trackIdx][idx] = -1; // Clear the detection out
                cost = statisticalDifferance(trackIdx, m_tracks, detectionToTest, m_dets);
                if (cost < minVal)
                {
                    minValIdx = detectionToTest;
                    minVal = cost;
                }
            }

            m_tracks.trackFiles[trackIdx].corrDet = minValIdx;
            m_dets.detList[m_tracks.trackFiles[trackIdx].corrDet].correlated = true;
            m_dets.detList[m_tracks.trackFiles[trackIdx].corrDet].corrTrack = trackIdx;

            trackHits[trackIdx] = 0; // Clear trackHits
        }
    }
}