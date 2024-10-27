#include "TrackFileMgr.hpp"

using namespace std;


int main()
{
    TrackFileMgr tracker_;

    // ifstream detections("detections.csv");
    ifstream detections("detections_from_video.csv");

    if (!detections.is_open())
    {
        std::cerr << "Error opening file for writing." << std::endl;
        return 1;
    }

    int frame;
    vector<double> xHold(2000, -1);
    vector<double> yHold(2000, -1);
    vector<double> frameHold(2000, -1);

    int idx = 0;
    int numFrames = -1;
    double frameValue, xValue, yValue;

    while (detections >> frameValue >> xValue >> yValue) {
        if (frameValue != numFrames)
        {
            numFrames = frameValue;
        }
        frameHold[idx] = frameValue;
        xHold[idx] = xValue;
        yHold[idx] = yValue;
        idx++;
    }
    numFrames++;

    // Define a simple dt to start each frame is dt = 1.0
    double dt = 1;

    for (int frame = 0; frame < numFrames; frame++)
    {
        int detIdx = 0;
        int detPerFrame = 0;
        for (int idx = 0; idx < frameHold.size(); idx++)
        {
            if (frameHold[idx] == frame)
            {
                detPerFrame++;
            }
        }
        // Assign DETECTIONS from input file
        for (int idx = 0; idx < frameHold.size(); idx++)
        {
            if (frameHold[idx] == frame)
            {
                tracker_.m_dets.detList[detIdx].pos[0] = xHold[idx];
                tracker_.m_dets.detList[detIdx].pos[1] = yHold[idx];
                tracker_.m_dets.detList[detIdx].valid  = VALID_DET;
                detIdx++;
            }
        }
        tracker_.m_dets.numDets = detIdx;
        /*
        >----------------------------------------------------------------------------
        |
        |   Start Track File manager functions.
        |
        |   1. Predict track location
        |   2. Associate Dets and Tracks
        |   3. Birth / Decay trackFiles
        |   4. Estimate new locations for those tracker_.m_tracks based on detection information 
        |
        >----------------------------------------------------------------------------
        */

        /*
        >----------------------------------------------------------------------------
        |
        |   .  INIT VARS
        |
        >----------------------------------------------------------------------------
        */
        tracker_.updateFrameVariables();

        /*
        >----------------------------------------------------------------------------
        |
        |   1.  Predict track location
        |
        >----------------------------------------------------------------------------
        */
        tracker_.predictTrackLocationAndGate(dt);

        /*
        >----------------------------------------------------------------------------
        |
        |   2. Associate Dets and Tracks
        |      CURRENTLY AUCTION IN BROKEN. IT RUNS FOREVER.
        |
        >----------------------------------------------------------------------------
        */
        tracker_.binningAssociate();
        // tracker_.hungarianAssociate();
        // for (int track = 0; track < tracker_.m_numActiveTracks; track++)
        // {
        //     int tf = tracker_.m_activeList[track];
        //     std::cout << std::setw(10) << tracker_.m_frame 
        //                 << "," << std::setw(10) << tracker_.m_tracks.trackFiles[tf].predPos[0] 
        //                     << "," << std::setw(10) << tracker_.m_tracks.trackFiles[tf].predPos[1] 
        //                         << "," << std::setw(10) << tracker_.m_tracks.trackFiles[tf].state 
        //                             << "," << std::setw(10) << tracker_.m_tracks.trackFiles[tf].corrDet 
        //                                 << std::endl;
        // }

        /*>----------------------------------------------------------------------------
        |
        |   3.  Update Track Position Estimate based on associated detection
        |
        >----------------------------------------------------------------------------*/
        tracker_.updateTrackEstPosition();

        /*>----------------------------------------------------------------------------
        |
        |   4.  Birth and Decay Track Files
        |
        >----------------------------------------------------------------------------*/
        tracker_.updateTrackVariables();

        // FRAME STATE
        for (int det = 0; det < tracker_.m_dets.numDets; det++)
        {
            std::cout << tracker_.m_frame << "," << 0 << "," << tracker_.m_dets.detList[det].pos[0] << "," << tracker_.m_dets.detList[det].pos[1] << "," << -1 << std::endl;
        }

        for (int track = 0; track < tracker_.m_numActiveTracks; track++)
        {
            int trackFile = tracker_.m_activeList[track];
            std::cout << tracker_.m_frame << ","
                         << 1 << "," 
                         << tracker_.m_tracks.trackFiles[trackFile].estState(0) << "," 
                         << tracker_.m_tracks.trackFiles[trackFile].estState(1) << "," 
                         << tracker_.m_tracks.trackFiles[trackFile].state << "," 
                         << tracker_.m_tracks.trackFiles[trackFile].estCov(0,0) << "," 
                         << tracker_.m_tracks.trackFiles[trackFile].estCov(1,1) << "," 
                         << std::endl;
        }


        tracker_.m_frame++;

        /*>----------------------------------------------------------------------------
        |
        |   5.  Frame clean up 
        |
        >----------------------------------------------------------------------------*/
        tracker_.frameCleanUp();
    }

    return 0;
}