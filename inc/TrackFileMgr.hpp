#ifndef TRACK_FILE_MGR_H
#define TRACK_FILE_MGR_H

#include "TrkUtility.hpp"

class TrackFileMgr {
public:
    TrackFileMgr();  // Constructor
    ~TrackFileMgr(); // Destructor

    uint16_t m_frame;

    void updateFrameVariables(TrackFile &tracks, DetList &dets);
    void predictTrackLocationAndGate(TrackFile tracks, double dt);
    void updateTrackEstPosition(TrackFile &tracks, DetList &dets);
    void correctTrackState(TrackFile tracks, double dt);
    void checkPersistency(TrackFile &tracks);
    void frameCleanUp(TrackFile &tracks, DetList &dets);
    void attemptOpenTracks(TrackFile &tracks, DetList &dets);
    void updateTrackVariables(TrackFile &tracks, DetList &dets);

    void binningAssociate(TrackFile &tracks, DetList &dets);
    
    void hungarianAssociate(
                            DetList &dets,
                            TrackFile &tracks
                            );

    void auctionAssociate(
                            DetList &dets,
                            TrackFile &tracks
                            );

};

#endif