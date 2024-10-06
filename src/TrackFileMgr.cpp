#include "TrackFileMgr.hpp"
#include <Eigen/Dense>

using namespace Eigen;

TrackFileMgr::TrackFileMgr()
: m_frame(0)
{
}

TrackFileMgr::~TrackFileMgr()
{
}

void TrackFileMgr::predictTrackLocationAndGate(TrackFile tracks, double dt)
{
    // STATE TRANSITION MATRIX F
        vector<vector<double>> F(4, vector<double>(4, 0.0));
        F[0][0] = 1.0;
        F[2][0] = dt;
        F[1][1] = 1.0;
        F[3][1] = dt;
        F[2][2] = 1.0;
        F[3][3] = 1.0;

        vector<vector<double>> Q(4, vector<double>(4, 0.0));
        Q[2][2] = ACCEL_STD*ACCEL_STD;
        Q[3][3] = ACCEL_STD*ACCEL_STD;

    // Loop over the tracks and predict the current location with constant velocity model
    for (int trkIdx = 0; trkIdx < TRACK_MAX; trkIdx++)
    {
        if (tracks.trackFiles[trkIdx].state != CLOSED)
        {   
            // State update equations. Position and Velocity. 
            tracks.trackFiles[trkIdx].predPos[0] = tracks.trackFiles[trkIdx].estPos[0] + tracks.trackFiles[trkIdx].estVel[0] * dt;
            tracks.trackFiles[trkIdx].predPos[1] = tracks.trackFiles[trkIdx].estPos[1] + tracks.trackFiles[trkIdx].estVel[1] * dt;

            tracks.trackFiles[trkIdx].predVel[0] = tracks.trackFiles[trkIdx].estVel[0];
            tracks.trackFiles[trkIdx].predVel[1] = tracks.trackFiles[trkIdx].estVel[1];

            // Predict the Covariance Matrix
            // COLUMN 0
            tracks.trackFiles[trkIdx].predCov[0][0] = tracks.trackFiles[trkIdx].estCov[0][0] + tracks.trackFiles[trkIdx].estCov[2][0] * dt
                    + tracks.trackFiles[trkIdx].estCov[0][2] * dt + tracks.trackFiles[trkIdx].estCov[2][2] * dt * dt;

            tracks.trackFiles[trkIdx].predCov[0][1] = tracks.trackFiles[trkIdx].estCov[0][1] + tracks.trackFiles[trkIdx].estCov[2][1] * dt
                    + tracks.trackFiles[trkIdx].estCov[0][3] * dt + tracks.trackFiles[trkIdx].estCov[2][3] * dt * dt;

            tracks.trackFiles[trkIdx].predCov[0][2] = tracks.trackFiles[trkIdx].estCov[0][2] + tracks.trackFiles[trkIdx].estCov[2][2] * dt;

            tracks.trackFiles[trkIdx].predCov[0][3] = tracks.trackFiles[trkIdx].estCov[0][3] + tracks.trackFiles[trkIdx].estCov[2][3] * dt;

            // COLUMN 1
            tracks.trackFiles[trkIdx].predCov[1][0] = tracks.trackFiles[trkIdx].estCov[1][0] + tracks.trackFiles[trkIdx].estCov[3][0] * dt
                    + tracks.trackFiles[trkIdx].estCov[1][2] * dt + tracks.trackFiles[trkIdx].estCov[3][2] * dt * dt;

            tracks.trackFiles[trkIdx].predCov[1][1] = tracks.trackFiles[trkIdx].estCov[1][1] + tracks.trackFiles[trkIdx].estCov[3][1] * dt
                    + tracks.trackFiles[trkIdx].estCov[1][3] * dt + tracks.trackFiles[trkIdx].estCov[3][3] * dt * dt;

            tracks.trackFiles[trkIdx].predCov[1][2] = tracks.trackFiles[trkIdx].estCov[1][2] + tracks.trackFiles[trkIdx].estCov[3][2] * dt;

            tracks.trackFiles[trkIdx].predCov[1][3] = tracks.trackFiles[trkIdx].estCov[1][3] + tracks.trackFiles[trkIdx].estCov[3][3] * dt;

            // COLUMN 2
            tracks.trackFiles[trkIdx].predCov[2][0] = tracks.trackFiles[trkIdx].estCov[2][0] + tracks.trackFiles[trkIdx].estCov[2][2] * dt;

            tracks.trackFiles[trkIdx].predCov[2][1] = tracks.trackFiles[trkIdx].estCov[2][1] + tracks.trackFiles[trkIdx].estCov[2][3] * dt;

            tracks.trackFiles[trkIdx].predCov[2][2] = tracks.trackFiles[trkIdx].estCov[2][2] + Q[2][2];

            tracks.trackFiles[trkIdx].predCov[2][3] = tracks.trackFiles[trkIdx].estCov[2][3];

            // COLUMN 3
            tracks.trackFiles[trkIdx].predCov[3][0] = tracks.trackFiles[trkIdx].estCov[3][0] + tracks.trackFiles[trkIdx].estCov[3][2] * dt;

            tracks.trackFiles[trkIdx].predCov[3][1] = tracks.trackFiles[trkIdx].estCov[3][1] + tracks.trackFiles[trkIdx].estCov[3][3] * dt;

            tracks.trackFiles[trkIdx].predCov[3][2] = tracks.trackFiles[trkIdx].estCov[3][2];

            tracks.trackFiles[trkIdx].predCov[3][3] = tracks.trackFiles[trkIdx].estCov[3][3] + Q[3][3];
        }
    }
}

void TrackFileMgr::updateTrackEstPosition(TrackFile &tracks, DetList &dets)
{
    for (int trkIdx = 0; trkIdx < TRACK_MAX; trkIdx++)
    {
        if (tracks.trackFiles[trkIdx].corrDet != -1)
        {

            // Update the track state estimate with the detection information
            
            // Compute the Kalman Gain
            vector<vector<double>> HPH_tPlusR(2, vector<double>(2, 0.0));
            HPH_tPlusR[0][0] = 1 / (tracks.trackFiles[trkIdx].predCov[0][0] + dets.detList[tracks.trackFiles[trkIdx].corrDet].measCov[0][0]);
            HPH_tPlusR[1][1] = 1 / (tracks.trackFiles[trkIdx].predCov[1][1] + dets.detList[tracks.trackFiles[trkIdx].corrDet].measCov[1][1]);

            vector<vector<double>> K(2 , vector<double>(4, 0.0));
            K[0][0] = HPH_tPlusR[0][0] * tracks.trackFiles[trkIdx].predCov[0][0];
            K[0][1] = HPH_tPlusR[0][0] * tracks.trackFiles[trkIdx].predCov[0][1];
            K[0][2] = HPH_tPlusR[0][0] * tracks.trackFiles[trkIdx].predCov[0][2];
            K[0][3] = HPH_tPlusR[0][0] * tracks.trackFiles[trkIdx].predCov[0][3];

            K[1][0] = HPH_tPlusR[1][1] * tracks.trackFiles[trkIdx].predCov[1][0];
            K[1][1] = HPH_tPlusR[1][1] * tracks.trackFiles[trkIdx].predCov[1][1];
            K[1][2] = HPH_tPlusR[1][1] * tracks.trackFiles[trkIdx].predCov[1][2];
            K[1][3] = HPH_tPlusR[1][1] * tracks.trackFiles[trkIdx].predCov[1][3];

            // // Calculate the residual
            double residualX = dets.detList[tracks.trackFiles[trkIdx].corrDet].pos[0] - tracks.trackFiles[trkIdx].predPos[0];
            double residualY = dets.detList[tracks.trackFiles[trkIdx].corrDet].pos[1] - tracks.trackFiles[trkIdx].predPos[1];

            // // Compute the Track Estimated Position
            tracks.trackFiles[trkIdx].estPos[0] = tracks.trackFiles[trkIdx].predPos[0] + K[0][0] * residualX + K[1][0] * residualY;
            tracks.trackFiles[trkIdx].estPos[1] = tracks.trackFiles[trkIdx].predPos[1] + K[0][1] * residualX + K[1][1] * residualY;
            tracks.trackFiles[trkIdx].estVel[0] = tracks.trackFiles[trkIdx].predVel[0] + K[0][2] * residualX + K[1][2] * residualY;
            tracks.trackFiles[trkIdx].estVel[1] = tracks.trackFiles[trkIdx].predVel[0] + K[0][3] * residualX + K[1][3] * residualY;

            // // Compute the Estimated Covariance Values
            // COL 0
            tracks.trackFiles[trkIdx].estCov[0][0] = (1 - K[0][0]) * tracks.trackFiles[trkIdx].predCov[0][0] +       K[1][0] * tracks.trackFiles[trkIdx].predCov[0][1];
            tracks.trackFiles[trkIdx].estCov[0][1] = K[0][1]       * tracks.trackFiles[trkIdx].predCov[0][0] + (1 - K[1][1]) * tracks.trackFiles[trkIdx].predCov[0][1];
            tracks.trackFiles[trkIdx].estCov[0][2] = K[0][2]       * tracks.trackFiles[trkIdx].predCov[0][0] +       K[1][2] * tracks.trackFiles[trkIdx].predCov[0][1] + tracks.trackFiles[trkIdx].predCov[0][2];
            tracks.trackFiles[trkIdx].estCov[0][3] = K[0][3]       * tracks.trackFiles[trkIdx].predCov[0][0] +       K[1][3] * tracks.trackFiles[trkIdx].predCov[0][1] + tracks.trackFiles[trkIdx].predCov[0][3];

            // COL 1
            tracks.trackFiles[trkIdx].estCov[1][0] = (1 - K[0][0]) * tracks.trackFiles[trkIdx].predCov[1][0] +       K[1][0] * tracks.trackFiles[trkIdx].predCov[1][1];
            tracks.trackFiles[trkIdx].estCov[1][1] = K[0][1]       * tracks.trackFiles[trkIdx].predCov[1][0] + (1 - K[1][1]) * tracks.trackFiles[trkIdx].predCov[1][1];
            tracks.trackFiles[trkIdx].estCov[1][2] = K[0][2]       * tracks.trackFiles[trkIdx].predCov[1][0] +       K[1][2] * tracks.trackFiles[trkIdx].predCov[1][1] + tracks.trackFiles[trkIdx].predCov[1][2];
            tracks.trackFiles[trkIdx].estCov[1][3] = K[0][3]       * tracks.trackFiles[trkIdx].predCov[1][0] +       K[1][3] * tracks.trackFiles[trkIdx].predCov[1][1] + tracks.trackFiles[trkIdx].predCov[1][3];

            // COL 2
            tracks.trackFiles[trkIdx].estCov[2][0] = (1 - K[0][0]) * tracks.trackFiles[trkIdx].predCov[2][0] +       K[1][0] * tracks.trackFiles[trkIdx].predCov[2][1];
            tracks.trackFiles[trkIdx].estCov[2][1] = K[0][1]       * tracks.trackFiles[trkIdx].predCov[2][0] + (1 - K[1][1]) * tracks.trackFiles[trkIdx].predCov[2][1];
            tracks.trackFiles[trkIdx].estCov[2][2] = K[0][2]       * tracks.trackFiles[trkIdx].predCov[2][0] +       K[1][2] * tracks.trackFiles[trkIdx].predCov[2][1] + tracks.trackFiles[trkIdx].predCov[2][2];
            tracks.trackFiles[trkIdx].estCov[2][3] = K[0][3]       * tracks.trackFiles[trkIdx].predCov[2][0] +       K[1][3] * tracks.trackFiles[trkIdx].predCov[2][1] + tracks.trackFiles[trkIdx].predCov[2][3];

            // COL 3
            tracks.trackFiles[trkIdx].estCov[3][0] = (1 - K[0][0]) * tracks.trackFiles[trkIdx].predCov[3][0] +       K[1][0] * tracks.trackFiles[trkIdx].predCov[3][1];
            tracks.trackFiles[trkIdx].estCov[3][1] = K[0][1]       * tracks.trackFiles[trkIdx].predCov[3][0] + (1 - K[1][1]) * tracks.trackFiles[trkIdx].predCov[3][1];
            tracks.trackFiles[trkIdx].estCov[3][2] = K[0][2]       * tracks.trackFiles[trkIdx].predCov[3][0] +       K[1][2] * tracks.trackFiles[trkIdx].predCov[3][1] + tracks.trackFiles[trkIdx].predCov[3][2];
            tracks.trackFiles[trkIdx].estCov[3][3] = K[0][3]       * tracks.trackFiles[trkIdx].predCov[3][0] +       K[1][3] * tracks.trackFiles[trkIdx].predCov[3][1] + tracks.trackFiles[trkIdx].predCov[3][3];
        }
    }
       
    
}

void TrackFileMgr::checkPersistency(TrackFile &tracks)
{
    for (int track = 0; track < TRACK_MAX; track++)
    {

        if (tracks.trackFiles[track].state != CLOSED)
        {
            // Update each trackFiles persistancy metric
            if ((tracks.trackFiles[track].corrDet != -1) && (tracks.trackFiles[track].persistance < 4))
            {
                tracks.trackFiles[track].persistance++;

                // Are we persistent enough to be converged 
                if (tracks.trackFiles[track].persistance > 2)
                {
                    tracks.trackFiles[track].state = CONVERGED;
                }
            }
            else if((tracks.trackFiles[track].corrDet == -1) && (tracks.trackFiles[track].persistance > 0))
            {
                tracks.trackFiles[track].persistance--;

                if (tracks.trackFiles[track].persistance == 0)
                {
                    tracks.trackFiles[track].reset();
                    tracks.numTracks--;
                }
            }
        }
    }
}

void TrackFileMgr::attemptOpenTracks(TrackFile &tracks, DetList &dets)
{
    // Loop through all valid dets
    for (int det = 0; det < DET_MAX; det++)
    {
        if (dets.detList[det].valid && !dets.detList[det].correlated)
        {
            for (int track = 0; track < TRACK_MAX; track++)
            {
                if (tracks.trackFiles[track].state == CLOSED)
                {
                    tracks.trackFiles[track].state = OPEN;
                    tracks.trackFiles[track].corrDet = det;
                    tracks.trackFiles[track].persistance;

                    tracks.trackFiles[track].estPos[0] = dets.detList[det].pos[0];
                    tracks.trackFiles[track].estPos[1] = dets.detList[det].pos[1];
                    tracks.trackFiles[track].predPos[0] = dets.detList[det].pos[0];
                    tracks.trackFiles[track].predPos[1] = dets.detList[det].pos[1];

                    tracks.trackFiles[track].predVel[0] = 0.0;
                    tracks.trackFiles[track].predVel[1] = 0.0;
                    tracks.trackFiles[track].estVel[0] = 0.0;
                    tracks.trackFiles[track].estVel[1] = 0.0;

                    tracks.trackFiles[track].predCov[0][0] = 5.0;
                    tracks.trackFiles[track].predCov[1][1] = 1.0;

                    tracks.trackFiles[track].estCov[0][0] = 5.0;
                    tracks.trackFiles[track].estCov[1][1] = 1.0;

                    tracks.numTracks++;
                    break;
                }
            }

            dets.numUncorrDets++;
        }
    }
}

void TrackFileMgr::updateFrameVariables(TrackFile &tracks, DetList &dets)
{
    // Clear the associations. The next frame needs the tracks to be a clean slate. 
    dets.numUncorrDets = 0;

}

void TrackFileMgr::frameCleanUp(TrackFile &tracks, DetList &dets)
{
    // Clear the associations. The next frame needs the tracks to be a clean slate. 
    for (int track = 0; track < TRACK_MAX; track++)
    {
        tracks.trackFiles[track].cleanCorrelated();
    }
    // Clear the Detection information
    for (int det = 0; det < DET_MAX; det++)
    {
        dets.detList[det].clearDet();
    }

}

void TrackFileMgr::updateTrackVariables(TrackFile &tracks, DetList &dets)
{   

    TrackFileMgr::attemptOpenTracks(tracks, dets);

    TrackFileMgr::checkPersistency(tracks);
}

void TrackFileMgr::correctTrackState(TrackFile tracks, double dt)
{
}