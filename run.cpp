#include "TrackFileMgr.hpp"

using namespace std;


int main()
{
    TrackFileMgr tracker_;

    ifstream detections("detections.csv");

    // cout << "a" << endl;
    if (!detections.is_open())
    {
        std::cerr << "Error opening file for writing." << std::endl;
        return 1;
    }

    vector<int>             state(STATE_MAX, -1);
    int frame;

    vector<double> xHold(DET_MAX, -1);
    vector<double> yHold(DET_MAX, -1);
    vector<double> frameHold(DET_MAX, -1);

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
        |   1.  Predict track location
        |
        >----------------------------------------------------------------------------
        */
        tracker_.predictTrackLocation(tracks, dt);

        /*
        >----------------------------------------------------------------------------
        |
        |   2. Associate Dets and Tracks
        |
        >----------------------------------------------------------------------------
        */
        cout << endl;
        tracker_.auctionAssociate(dets, tracks, state, dets.numDets, tracks.numTracks);
        print_state(state);

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

        /*>----------------------------------------------------------------------------
        |
        |   5.  Frame clean up 
        |
        >----------------------------------------------------------------------------*/
        tracker_.frameCleanUp(tracks, dets);
    }

    return 0;
}