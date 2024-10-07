#include "TrackFileMgr.hpp"

using namespace std;


int main()
{
    TrackFileMgr tracker_;

    ifstream detections("detections.csv");

    if (!detections.is_open())
    {
        std::cerr << "Error opening file for writing." << std::endl;
        return 1;
    }

    int frame;
    vector<double> xHold(200, -1);
    vector<double> yHold(200, -1);
    vector<double> frameHold(200, -1);

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
    double dt = 0.1;

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
        cout << endl;
        // tracker_.binningAssociate(tracker_.m_tracks, dets);
        tracker_.hungarianAssociate();
        // print_state(state);

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
        std::cout << "Frame: " << tracker_.m_frame++ << std::endl;
        for (int track = 0; track < 20; track++)
        {
            // if (tracker_.m_tracks.trackFiles[track].state != CLOSED)
            // {
                std::cout << "Track: " << track << ". Correlated?: " << tracker_.m_tracks.trackFiles[track].corrDet << ". Status: " << tracker_.m_tracks.trackFiles[track].state 
                    << ". Estimated Position: " << tracker_.m_tracks.trackFiles[track].estPos[0] << ". " << tracker_.m_tracks.trackFiles[track].estPos[1] << ". Estimated Vel " << tracker_.m_tracks.trackFiles[track].estVel[0] << ". " << tracker_.m_tracks.trackFiles[track].estVel[1] << std::endl;
            // }
            
        }

        /*>----------------------------------------------------------------------------
        |
        |   5.  Frame clean up 
        |
        >----------------------------------------------------------------------------*/
        tracker_.frameCleanUp();
    }

    return 0;
}