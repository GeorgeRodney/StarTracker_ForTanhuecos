#ifndef TRACK_FILE_MGR_H
#define TRACK_FILE_MGR_H

#include "TrkUtility.hpp"

class TrackFileMgr {
public:
    TrackFileMgr();  // Constructor
    ~TrackFileMgr(); // Destructor

    uint16_t m_frame;
    TrackFile m_tracks;
    DetList m_dets;

    void updateFrameVariables();
    void predictTrackLocationAndGate(double dt);
    void updateTrackEstPosition();
    void correctTrackState(double dt);
    void checkPersistency();
    void frameCleanUp();
    void attemptOpenTracks();
    void updateTrackVariables();

    void binningAssociate();
    
    void hungarianAssociate();

    void auctionAssociate();

};

#endif