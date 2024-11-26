#include "TrackFileMgr.hpp"

void TrackFileMgr::binningAssociate()
{
    // DONT RUN IF THERE IS NOTHING TO DO
    if ((m_tracks.numTracks == 0) || (m_dets.numDets == 0))
    {
        return;
    }

    vector<vector<int>> tracksToDets(TRACK_MAX, vector<int>(DET_MAX, -1));
    vector<vector<int>> detsToTracks(DET_MAX, vector<int>(TRACK_MAX, -1));
    vector<int> trackHits(TRACK_MAX, 0);
    vector<int> detHits(DET_MAX, 0);
    int trkHitIdx = 0;
    int detHitIdx = 0;
    double cost;
    double minVal;
    int minValIdx = -1;

    // Gate the m_tracks
    for(int16_t trackIdx = 0; trackIdx < m_numActiveTracks; trackIdx++)
    {
        int tf = m_activeList[trackIdx];
        for (int16_t detIdx = 0; detIdx < m_dets.numDets; detIdx++)
        {
            // CALCULATE GATE
            double diff = euclidean(tf, m_tracks, detIdx, m_dets);
            // double diff = statisticalDifferance(tf, m_tracks, detIdx, m_dets);
            // std::cout << "FRAME: " << m_frame << std::setw(10)
            //             << "cost: "  << diff << std::setw(10)
            //                 << "det: " << detIdx << std::setw(10)
            //                     << "track: " << trackIdx << std::setw(10)
            //                         << std::endl;

            if (diff < m_tracks.trackFiles[tf].gate)
            {
                // Store track stuff
                trackHits[tf]++;
                for (int trkHitIdx = 0; trkHitIdx < m_dets.numDets; trkHitIdx++)
                {
                    if (tracksToDets[tf][trkHitIdx] == -1)
                    {
                        tracksToDets[tf][trkHitIdx] = detIdx;
                        break;
                    }
                }
                
                // Store Det Info
                detHits[detIdx]++;
                for (int detHitIdx = 0; detHitIdx < TRACK_MAX; detHitIdx++)
                {
                    if (detsToTracks[detIdx][detHitIdx] == -1)
                    {
                        detsToTracks[detIdx][detHitIdx] = tf;
                        break;
                    }
                }
            }
        }
    }

    // ASSOCIATE
    for(int16_t trackIdx = 0; trackIdx < m_numActiveTracks; trackIdx++)
    {   
        int tf = m_activeList[trackIdx];
        minVal = 100000.0;
        // Associate the m_tracks with one detection in their gate
        if(trackHits[tf] == 1)
        {
            int temp_det = tracksToDets[tf][0];
            m_tracks.trackFiles[tf].corrDet = temp_det;
            m_dets.detList[m_tracks.trackFiles[tf].corrDet].correlated = true;
            m_dets.detList[m_tracks.trackFiles[tf].corrDet].corrTrack = tf;

            // Attach the truth ID
            m_tracks.trackFiles[tf].truth_id = m_dets.detList[temp_det].truth_id;

            // Remove from lists
            trackHits[tf] = 0;
            tracksToDets[tf][0] = -1;
            
            // Remove this detection from all the track lists
            removeDetFromLists(tracksToDets, trackHits, temp_det);


        }

        // Associate track with detection with the smallest cost
        if (trackHits[tf] > 1)
        {
            for(int idx = 0; idx < trackHits[tf]; idx++)
            {
                int detectionToTest = tracksToDets[tf][idx];
                tracksToDets[tf][idx] = -1; // Clear the detection out
                cost = euclidean(tf, m_tracks, detectionToTest, m_dets);
                // cost = statisticalDifferance(tf, m_tracks, detectionToTest, m_dets);
                if (cost < minVal)
                {
                    minValIdx = detectionToTest;
                    minVal = cost;
                }
            }

            m_tracks.trackFiles[tf].corrDet = minValIdx;
            m_dets.detList[m_tracks.trackFiles[tf].corrDet].correlated = true;
            m_dets.detList[m_tracks.trackFiles[tf].corrDet].corrTrack = tf;

            // Attach the truth ID
            m_tracks.trackFiles[tf].truth_id = m_dets.detList[minValIdx].truth_id;

            trackHits[tf] = 0; // Clear trackHits
        }
    }
}

void TrackFileMgr::removeDetFromLists(vector<vector<int>> &tracksToDets, vector<int> &trackHits, int temp_det)
{
    for (int trk = 0; trk < m_numActiveTracks; trk++)
    {
        int tf_ = m_activeList[trk];
        for (int det = 0; det < m_dets.numDets; det++)
        {
            if (tracksToDets[tf_][det] == temp_det)
            {
                tracksToDets[tf_][det] = -1;
                trackHits[tf_]--;
            }
        }
    }
}