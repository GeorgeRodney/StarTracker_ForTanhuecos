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

    vector<int> state(STATE_MAX, -1);
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

    DetList dets;
    TrackFile tracks;

    // Define a simple dt to start each frame is dt = 1.0
    double dt = 1.0;

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
                dets.detList[detIdx].pos[0] = xHold[idx];
                dets.detList[detIdx].pos[1] = yHold[idx];
                dets.detList[detIdx].valid  = VALID_DET;
                detIdx++;
            }
        }
        dets.numDets = detIdx;
        /*
        >----------------------------------------------------------------------------
        |
        |   Start Track File manager functions.
        |
        |   1. Predict track location
        |   2. Associate Dets and Tracks
        |   3. Birth / Decay trackFiles
        |   4. Estimate new locations for those tracks based on detection information 
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
        tracker_.updateFrameVariables(tracks, dets);

        /*
        >----------------------------------------------------------------------------
        |
        |   1.  Predict track location
        |
        >----------------------------------------------------------------------------
        */
        tracker_.predictTrackLocationAndGate(tracks, dt);

        /*
        >----------------------------------------------------------------------------
        |
        |   2. Associate Dets and Tracks
        |      CURRENTLY AUCTION IN BROKEN. IT RUNS FOREVER.
        |
        >----------------------------------------------------------------------------
        */
        cout << endl;
        tracker_.correlateDetsAndTracks(tracks, dets);
        // tracker_.hungarianAssociate(dets, tracks, state, dets.numDets, tracks.numTracks);
        // print_state(state);

        /*>----------------------------------------------------------------------------
        |
        |   3.  Update Track Position Estimate based on associated detection
        |
        >----------------------------------------------------------------------------*/
        tracker_.updateTrackEstPosition(tracks, dets);

        /*>----------------------------------------------------------------------------
        |
        |   4.  Birth and Decay Track Files
        |
        >----------------------------------------------------------------------------*/
        tracker_.updateTrackVariables(tracks, dets);

        // FRAME STATE
        std::cout << "Frame: " << tracker_.m_frame++ << std::endl;
        for (int track = 0; track < TRACK_MAX; track++)
        {
            if (tracks.trackFiles[track].state != CLOSED)
            {
                std::cout << "Track: " << track << ". Correlated?: " << tracks.trackFiles[track].corrDet << ". Status: " << tracks.trackFiles[track].state 
                    << ". Estimated Position: " << tracks.trackFiles[track].estPos[0] << ". " << tracks.trackFiles[track].estPos[1] << std::endl;
            }
            
        }

        /*>----------------------------------------------------------------------------
        |
        |   5.  Frame clean up 
        |
        >----------------------------------------------------------------------------*/
        tracker_.frameCleanUp(tracks, dets);
    }

    return 0;
}