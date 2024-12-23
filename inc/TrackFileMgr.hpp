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
    uint16_t m_numActiveTracks;
    vector<int> m_activeList;
    Eigen::MatrixXi m_truthToTrack;
    vector<PerfTruth> m_perfTruth;

    void updateFrameVariables();
    void predictTrackLocation(double dt);
    void updateTrackEstPosition();
    void correctTrackState(double dt);
    void checkPersistency();
    void frameCleanUp();
    void attemptOpenTracks();
    void fillPerformanceArray();
    void updateTrackVariables();
    void modifyActiveList();

    void binningAssociate();
    void removeDetFromLists(vector<vector<int>> &tracksToDets, vector<int> &trackHits, int temp_det);
    
    void hungarianAssociate();

    void auctionAssociate();

};

#endif