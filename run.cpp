#include "TrackFileMgr.hpp"
#include "PerformanceVacuum.hpp"

// Track Info Publishing
#include <chrono>

using namespace std;


int main()
{
    TrackFileMgr tracker_;
    // AsyncTrackingMetricsPublisher metrics_publisher_;

    ifstream detections("detections.csv");
    // ifstream detections("detections_from_video.csv");

    if (!detections.is_open())
    {
        std::cerr << "Error opening file for writing." << std::endl;
        return 1;
    }

    int frame;
    vector<double> xHold(2000, -1);
    vector<double> yHold(2000, -1);
    vector<int>    truthHold(2000, -1);
    vector<double> frameHold(2000, -1);

    int idx = 0;
    int numFrames = -1;
    double frameValue, xValue, yValue, truthId;

    while (detections >> frameValue >> xValue >> yValue >> truthId) {
        if (frameValue != numFrames)
        {
            numFrames = frameValue;
        }
        frameHold[idx] = frameValue;
        xHold[idx] = xValue;
        yHold[idx] = yValue;
        truthHold[idx] = truthId;
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
                tracker_.m_dets.detList[detIdx].truth_id = truthHold[idx];
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

        auto start_time = std::chrono::high_resolution_clock::now();

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
        tracker_.predictTrackLocation(dt);

        /*
        >----------------------------------------------------------------------------
        |
        |   2. Associate Dets and Tracks
        |      CURRENTLY AUCTION IN BROKEN. IT RUNS FOREVER.
        |
        >----------------------------------------------------------------------------
        */
        tracker_.binningAssociate();

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
        auto end_time = std::chrono::high_resolution_clock::now();
        double processing_time = std::chrono::duration<double>(end_time - start_time).count();

        /*>----------------------------------------------------------------------------
        |
        |   5.  Frame clean up / send data for performance
        |
        >----------------------------------------------------------------------------*/
        tracker_.fillPerformanceArray();
        // metrics_publisher_.queue_frame_data(tracker_.m_frame, track_associations, processing_time);
        for (int det = 0; det < tracker_.m_dets.numDets; det++)
        {
            std::cout << tracker_.m_frame << "," << 0 << "," << tracker_.m_dets.detList[det].pos[0] << "," << tracker_.m_dets.detList[det].pos[1] << "," << -1 << std::endl;
        }

        for (int track = 0; track < tracker_.m_numActiveTracks; track++)
        {
            int trackFile = tracker_.m_activeList[track];

            std::cout   << tracker_.m_frame << ","
                        << 1 << "," 
                        << tracker_.m_tracks.trackFiles[trackFile].estState(0) << "," 
                        << tracker_.m_tracks.trackFiles[trackFile].estState(1) << ","                          
                        << tracker_.m_tracks.trackFiles[trackFile].state << "," 
                        << tracker_.m_tracks.trackFiles[trackFile].estCov(0,0) << "," 
                        << tracker_.m_tracks.trackFiles[trackFile].estCov(1,1) << "," 
                        << tracker_.m_tracks.trackFiles[trackFile].uniqueId
                        << std::endl;
        }

        tracker_.m_frame++;
        tracker_.frameCleanUp();
    }

    return 0;
}