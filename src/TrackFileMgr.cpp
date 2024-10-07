#include "TrackFileMgr.hpp"
#include <Eigen/Dense>

using namespace Eigen;

TrackFileMgr::TrackFileMgr()
:   m_frame(0),
    m_tracks(),
    m_dets()
{
}

TrackFileMgr::~TrackFileMgr()
{
}

void TrackFileMgr::predictTrackLocationAndGate(double dt)
{
    // STATE TRANSITION MATRIX F
        vector<vector<double>> F(4, vector<double>(4, 0.0));
        F[0][0] = 1.0;
        F[2][0] = dt;
        F[1][1] = 1.0;
        F[3][1] = dt;
        F[2][2] = 1.0;
        F[3][3] = 1.0;

        double process_noise = 0.1;
        vector<vector<double>> Q(4, vector<double>(4, 0.0));
        Q[0][0] = process_noise * dt*dt*dt*dt / 4;
        Q[0][2] = process_noise * dt*dt*dt / 2;
        Q[1][1] = process_noise * dt*dt*dt*dt / 4;
        Q[1][3] = process_noise * dt*dt*dt / 2;
        Q[2][0] = process_noise * dt*dt*dt / 2;
        Q[2][2] = process_noise * dt*dt;
        Q[3][1] = process_noise * dt*dt*dt / 2;
        Q[3][3] = process_noise * dt*dt;

    // Loop over the m_tracks and predict the current location with constant velocity model
    for (int trkIdx = 0; trkIdx < TRACK_MAX; trkIdx++)
    {
        if (m_tracks.trackFiles[trkIdx].state != CLOSED)
        {   
            // State update equations. Position and Velocity. 
            m_tracks.trackFiles[trkIdx].predPos[0] = m_tracks.trackFiles[trkIdx].estPos[0] + m_tracks.trackFiles[trkIdx].estVel[0] * dt;
            m_tracks.trackFiles[trkIdx].predPos[1] = m_tracks.trackFiles[trkIdx].estPos[1] + m_tracks.trackFiles[trkIdx].estVel[1] * dt;

            m_tracks.trackFiles[trkIdx].predVel[0] = m_tracks.trackFiles[trkIdx].estVel[0];
            m_tracks.trackFiles[trkIdx].predVel[1] = m_tracks.trackFiles[trkIdx].estVel[1];

            // Predict the Covariance Matrix
            // COLUMN 0
            m_tracks.trackFiles[trkIdx].predCov[0][0] = m_tracks.trackFiles[trkIdx].estCov[0][0] + m_tracks.trackFiles[trkIdx].estCov[2][0] * dt
                    + m_tracks.trackFiles[trkIdx].estCov[0][2] * dt + m_tracks.trackFiles[trkIdx].estCov[2][2] * dt * dt + Q[0][0];

            m_tracks.trackFiles[trkIdx].predCov[0][1] = m_tracks.trackFiles[trkIdx].estCov[0][1] + m_tracks.trackFiles[trkIdx].estCov[2][1] * dt
                    + m_tracks.trackFiles[trkIdx].estCov[0][3] * dt + m_tracks.trackFiles[trkIdx].estCov[2][3] * dt * dt;

            m_tracks.trackFiles[trkIdx].predCov[0][2] = m_tracks.trackFiles[trkIdx].estCov[0][2] + m_tracks.trackFiles[trkIdx].estCov[2][2] * dt + Q[0][2];

            m_tracks.trackFiles[trkIdx].predCov[0][3] = m_tracks.trackFiles[trkIdx].estCov[0][3] + m_tracks.trackFiles[trkIdx].estCov[2][3] * dt;

            // COLUMN 1
            m_tracks.trackFiles[trkIdx].predCov[1][0] = m_tracks.trackFiles[trkIdx].estCov[1][0] + m_tracks.trackFiles[trkIdx].estCov[3][0] * dt
                    + m_tracks.trackFiles[trkIdx].estCov[1][2] * dt + m_tracks.trackFiles[trkIdx].estCov[3][2] * dt * dt;

            m_tracks.trackFiles[trkIdx].predCov[1][1] = m_tracks.trackFiles[trkIdx].estCov[1][1] + m_tracks.trackFiles[trkIdx].estCov[3][1] * dt
                    + m_tracks.trackFiles[trkIdx].estCov[1][3] * dt + m_tracks.trackFiles[trkIdx].estCov[3][3] * dt * dt + Q[1][1];

            m_tracks.trackFiles[trkIdx].predCov[1][2] = m_tracks.trackFiles[trkIdx].estCov[1][2] + m_tracks.trackFiles[trkIdx].estCov[3][2] * dt;

            m_tracks.trackFiles[trkIdx].predCov[1][3] = m_tracks.trackFiles[trkIdx].estCov[1][3] + m_tracks.trackFiles[trkIdx].estCov[3][3] * dt + Q[1][3];

            // COLUMN 2
            m_tracks.trackFiles[trkIdx].predCov[2][0] = m_tracks.trackFiles[trkIdx].estCov[2][0] + m_tracks.trackFiles[trkIdx].estCov[2][2] * dt + Q[2][0];

            m_tracks.trackFiles[trkIdx].predCov[2][1] = m_tracks.trackFiles[trkIdx].estCov[2][1] + m_tracks.trackFiles[trkIdx].estCov[2][3] * dt;

            m_tracks.trackFiles[trkIdx].predCov[2][2] = m_tracks.trackFiles[trkIdx].estCov[2][2] + Q[2][2];

            m_tracks.trackFiles[trkIdx].predCov[2][3] = m_tracks.trackFiles[trkIdx].estCov[2][3];

            // COLUMN 3
            m_tracks.trackFiles[trkIdx].predCov[3][0] = m_tracks.trackFiles[trkIdx].estCov[3][0] + m_tracks.trackFiles[trkIdx].estCov[3][2] * dt;

            m_tracks.trackFiles[trkIdx].predCov[3][1] = m_tracks.trackFiles[trkIdx].estCov[3][1] + m_tracks.trackFiles[trkIdx].estCov[3][3] * dt + Q[3][1];

            m_tracks.trackFiles[trkIdx].predCov[3][2] = m_tracks.trackFiles[trkIdx].estCov[3][2];

            m_tracks.trackFiles[trkIdx].predCov[3][3] = m_tracks.trackFiles[trkIdx].estCov[3][3] + Q[3][3];
        }
    }
}

void TrackFileMgr::updateTrackEstPosition()
{
    for (int trkIdx = 0; trkIdx < TRACK_MAX; trkIdx++)
    {
        if (m_tracks.trackFiles[trkIdx].corrDet != -1)
        {
            // Update the track state estimate with the detection information
            
            // Compute the Kalman Gain
            vector<vector<double>> HPH_tPlusR(2, vector<double>(2, 0.0));
            HPH_tPlusR[0][0] = 1 / (m_tracks.trackFiles[trkIdx].predCov[0][0] + m_dets.detList[m_tracks.trackFiles[trkIdx].corrDet].measCov[0][0]);
            HPH_tPlusR[1][1] = 1 / (m_tracks.trackFiles[trkIdx].predCov[1][1] + m_dets.detList[m_tracks.trackFiles[trkIdx].corrDet].measCov[1][1]);

            vector<vector<double>> K(2 , vector<double>(4, 0.0));
            K[0][0] = HPH_tPlusR[0][0] * m_tracks.trackFiles[trkIdx].predCov[0][0];
            K[0][1] = HPH_tPlusR[0][0] * m_tracks.trackFiles[trkIdx].predCov[0][1];
            K[0][2] = HPH_tPlusR[0][0] * m_tracks.trackFiles[trkIdx].predCov[0][2];
            K[0][3] = HPH_tPlusR[0][0] * m_tracks.trackFiles[trkIdx].predCov[0][3];

            K[1][0] = HPH_tPlusR[1][1] * m_tracks.trackFiles[trkIdx].predCov[1][0];
            K[1][1] = HPH_tPlusR[1][1] * m_tracks.trackFiles[trkIdx].predCov[1][1];
            K[1][2] = HPH_tPlusR[1][1] * m_tracks.trackFiles[trkIdx].predCov[1][2];
            K[1][3] = HPH_tPlusR[1][1] * m_tracks.trackFiles[trkIdx].predCov[1][3];

            // // Calculate the residual
            double residualX = m_dets.detList[m_tracks.trackFiles[trkIdx].corrDet].pos[0] - m_tracks.trackFiles[trkIdx].predPos[0];
            double residualY = m_dets.detList[m_tracks.trackFiles[trkIdx].corrDet].pos[1] - m_tracks.trackFiles[trkIdx].predPos[1];

            // // Compute the Track Estimated Position
            m_tracks.trackFiles[trkIdx].estPos[0] = m_tracks.trackFiles[trkIdx].predPos[0] + K[0][0] * residualX + K[1][0] * residualY;
            m_tracks.trackFiles[trkIdx].estPos[1] = m_tracks.trackFiles[trkIdx].predPos[1] + K[0][1] * residualX + K[1][1] * residualY;
            m_tracks.trackFiles[trkIdx].estVel[0] = m_tracks.trackFiles[trkIdx].predVel[0] + K[0][2] * residualX + K[1][2] * residualY;
            m_tracks.trackFiles[trkIdx].estVel[1] = m_tracks.trackFiles[trkIdx].predVel[1] + K[0][3] * residualX + K[1][3] * residualY;

            // // Compute the Estimated Covariance Values
            // COL 0
            m_tracks.trackFiles[trkIdx].estCov[0][0] = (1 - K[0][0]) * m_tracks.trackFiles[trkIdx].predCov[0][0] +       K[1][0] * m_tracks.trackFiles[trkIdx].predCov[0][1];
            m_tracks.trackFiles[trkIdx].estCov[0][1] = K[0][1]       * m_tracks.trackFiles[trkIdx].predCov[0][0] + (1 - K[1][1]) * m_tracks.trackFiles[trkIdx].predCov[0][1];
            m_tracks.trackFiles[trkIdx].estCov[0][2] = K[0][2]       * m_tracks.trackFiles[trkIdx].predCov[0][0] +       K[1][2] * m_tracks.trackFiles[trkIdx].predCov[0][1] + m_tracks.trackFiles[trkIdx].predCov[0][2];
            m_tracks.trackFiles[trkIdx].estCov[0][3] = K[0][3]       * m_tracks.trackFiles[trkIdx].predCov[0][0] +       K[1][3] * m_tracks.trackFiles[trkIdx].predCov[0][1] + m_tracks.trackFiles[trkIdx].predCov[0][3];

            // COL 1
            m_tracks.trackFiles[trkIdx].estCov[1][0] = (1 - K[0][0]) * m_tracks.trackFiles[trkIdx].predCov[1][0] +       K[1][0] * m_tracks.trackFiles[trkIdx].predCov[1][1];
            m_tracks.trackFiles[trkIdx].estCov[1][1] = K[0][1]       * m_tracks.trackFiles[trkIdx].predCov[1][0] + (1 - K[1][1]) * m_tracks.trackFiles[trkIdx].predCov[1][1];
            m_tracks.trackFiles[trkIdx].estCov[1][2] = K[0][2]       * m_tracks.trackFiles[trkIdx].predCov[1][0] +       K[1][2] * m_tracks.trackFiles[trkIdx].predCov[1][1] + m_tracks.trackFiles[trkIdx].predCov[1][2];
            m_tracks.trackFiles[trkIdx].estCov[1][3] = K[0][3]       * m_tracks.trackFiles[trkIdx].predCov[1][0] +       K[1][3] * m_tracks.trackFiles[trkIdx].predCov[1][1] + m_tracks.trackFiles[trkIdx].predCov[1][3];

            // COL 2
            m_tracks.trackFiles[trkIdx].estCov[2][0] = (1 - K[0][0]) * m_tracks.trackFiles[trkIdx].predCov[2][0] +       K[1][0] * m_tracks.trackFiles[trkIdx].predCov[2][1];
            m_tracks.trackFiles[trkIdx].estCov[2][1] = K[0][1]       * m_tracks.trackFiles[trkIdx].predCov[2][0] + (1 - K[1][1]) * m_tracks.trackFiles[trkIdx].predCov[2][1];
            m_tracks.trackFiles[trkIdx].estCov[2][2] = K[0][2]       * m_tracks.trackFiles[trkIdx].predCov[2][0] +       K[1][2] * m_tracks.trackFiles[trkIdx].predCov[2][1] + m_tracks.trackFiles[trkIdx].predCov[2][2];
            m_tracks.trackFiles[trkIdx].estCov[2][3] = K[0][3]       * m_tracks.trackFiles[trkIdx].predCov[2][0] +       K[1][3] * m_tracks.trackFiles[trkIdx].predCov[2][1] + m_tracks.trackFiles[trkIdx].predCov[2][3];

            // COL 3
            m_tracks.trackFiles[trkIdx].estCov[3][0] = (1 - K[0][0]) * m_tracks.trackFiles[trkIdx].predCov[3][0] +       K[1][0] * m_tracks.trackFiles[trkIdx].predCov[3][1];
            m_tracks.trackFiles[trkIdx].estCov[3][1] = K[0][1]       * m_tracks.trackFiles[trkIdx].predCov[3][0] + (1 - K[1][1]) * m_tracks.trackFiles[trkIdx].predCov[3][1];
            m_tracks.trackFiles[trkIdx].estCov[3][2] = K[0][2]       * m_tracks.trackFiles[trkIdx].predCov[3][0] +       K[1][2] * m_tracks.trackFiles[trkIdx].predCov[3][1] + m_tracks.trackFiles[trkIdx].predCov[3][2];
            m_tracks.trackFiles[trkIdx].estCov[3][3] = K[0][3]       * m_tracks.trackFiles[trkIdx].predCov[3][0] +       K[1][3] * m_tracks.trackFiles[trkIdx].predCov[3][1] + m_tracks.trackFiles[trkIdx].predCov[3][3];
        }
    }
       
    
}

void TrackFileMgr::checkPersistency()
{
    for (int track = 0; track < TRACK_MAX; track++)
    {

        if (m_tracks.trackFiles[track].state != CLOSED)
        {
            // Update each trackFiles persistancy metric
            if ((m_tracks.trackFiles[track].corrDet != -1) && (m_tracks.trackFiles[track].persistance < 4))
            {
                m_tracks.trackFiles[track].persistance++;

                // Are we persistent enough to be converged 
                if (m_tracks.trackFiles[track].persistance > 2)
                {
                    m_tracks.trackFiles[track].state = CONVERGED;
                }
            }
            else if((m_tracks.trackFiles[track].corrDet == -1) && (m_tracks.trackFiles[track].persistance > 0))
            {
                m_tracks.trackFiles[track].persistance--;

                if (m_tracks.trackFiles[track].persistance == 0)
                {
                    m_tracks.trackFiles[track].state = CLOSED;
                    m_tracks.trackFiles[track].reset();
                    m_tracks.numTracks--;
                }
            }
        }
    }
}

void TrackFileMgr::attemptOpenTracks()
{
    // Loop through all valid m_dets
    for (int det = 0; det < DET_MAX; det++)
    {
        if (m_dets.detList[det].valid && !m_dets.detList[det].correlated)
        {
            for (int track = 0; track < TRACK_MAX; track++)
            {
                if (m_tracks.trackFiles[track].state == CLOSED)
                {
                    m_tracks.trackFiles[track].state = OPEN;
                    m_tracks.trackFiles[track].corrDet = det;
                    m_tracks.trackFiles[track].persistance;

                    m_tracks.trackFiles[track].estPos[0] = m_dets.detList[det].pos[0];
                    m_tracks.trackFiles[track].estPos[1] = m_dets.detList[det].pos[1];
                    m_tracks.trackFiles[track].predPos[0] = m_dets.detList[det].pos[0];
                    m_tracks.trackFiles[track].predPos[1] = m_dets.detList[det].pos[1];

                    m_tracks.trackFiles[track].predVel[0] = 0.0;
                    m_tracks.trackFiles[track].predVel[1] = 0.0;
                    m_tracks.trackFiles[track].estVel[0] = 0.0;
                    m_tracks.trackFiles[track].estVel[1] = 0.0;

                    m_tracks.trackFiles[track].predCov[0][0] = 5.0;
                    m_tracks.trackFiles[track].predCov[1][1] = 1.0;

                    m_tracks.trackFiles[track].estCov[0][0] = 5.0;
                    m_tracks.trackFiles[track].estCov[1][1] = 1.0;

                    m_tracks.numTracks++;
                    break;
                }
            }

            m_dets.numUncorrDets++;
        }
    }
}

void TrackFileMgr::updateFrameVariables()
{
    // Clear the associations. The next frame needs the m_tracks to be a clean slate. 
    m_dets.numUncorrDets = 0;

}

void TrackFileMgr::frameCleanUp()
{
    // Clear the associations. The next frame needs the m_tracks to be a clean slate. 
    for (int track = 0; track < TRACK_MAX; track++)
    {
        m_tracks.trackFiles[track].cleanCorrelated();
    }
    // Clear the Detection information
    for (int det = 0; det < DET_MAX; det++)
    {
        m_dets.detList[det].clearDet();
    }

}

void TrackFileMgr::updateTrackVariables()
{   
        TrackFileMgr::checkPersistency();

    TrackFileMgr::attemptOpenTracks();
}

void TrackFileMgr::correctTrackState(double dt)
{
}