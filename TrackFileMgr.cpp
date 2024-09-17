#include "TrackFileMgr.hpp"
#include <Eigen/Dense>

using namespace Eigen;

TrackFileMgr::TrackFileMgr()
{
    // Constructor implementation
}

TrackFileMgr::~TrackFileMgr()
{
    // Destructor implementation
}

void TrackFileMgr::predictTrackLocation(TrackFile tracks, double dt)
{
    // STATE TRANSITION MATRIX F
        MatrixXd F = Matrix4d();
        F <<    1,0,dt,0,
                0,1,0 ,dt,
                0,0,1 ,0,
                0,0,0 ,1;

        MatrixXd Q = Matrix4d::Zero();
        Q(1,1) = (ACCEL_STD*ACCEL_STD);
        Q(3,3) = (ACCEL_STD*ACCEL_STD);

    // Loop over the tracks and predict the current location with constant velocity model
    for (int trkIdx = 0; trkIdx < TRACK_MAX; trkIdx++)
    {
        

        if (tracks.trackFiles[trkIdx].state != CLOSED)
        {   
            // State update equations. Position and Velocity. 
            tracks.trackFiles[trkIdx].predPos[0] = tracks.trackFiles[trkIdx].estPos[0] * F(0,0) + 
                                                        tracks.trackFiles[trkIdx].vel[0] * F(0,2);
            tracks.trackFiles[trkIdx].predPos[1] = tracks.trackFiles[trkIdx].estPos[1] * F(1,1) + 
                                                        tracks.trackFiles[trkIdx].vel[1] * F(1,3);

            tracks.trackFiles[trkIdx].vel[0] = tracks.trackFiles[trkIdx].vel[0] * F(2,2);
            tracks.trackFiles[trkIdx].vel[1] = tracks.trackFiles[trkIdx].vel[1] * F(3,3);

            // Predict the Covariance Matrix
            tracks.trackFiles[trkIdx].predCov = F * tracks.trackFiles[trkIdx].estCov * F.transpose() + Q;
        }
    }
}

void TrackFileMgr::updateTrackEstPosition(TrackFile &tracks, DetList &dets)
{
    
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
                if (tracks.trackFiles[track].persistance > 3)
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
        // else if (tracks.trackFiles[track].state == CONVERGED)
        // {

        // }
        // else
        // {

        // }
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
                    tracks.trackFiles[track].persistance++;

                    tracks.trackFiles[track].estPos[0] = dets.detList[det].pos[0];
                    tracks.trackFiles[track].estPos[1] = dets.detList[det].pos[1];
                    tracks.trackFiles[track].predPos[0] = dets.detList[det].pos[0];
                    tracks.trackFiles[track].predPos[1] = dets.detList[det].pos[1];

                    tracks.trackFiles[track].predCov(0,0) = 5.0;
                    tracks.trackFiles[track].predCov(1,1) = 1.0;

                    tracks.trackFiles[track].estCov(0,0) = 5.0;
                    tracks.trackFiles[track].estCov(1,1) = 1.0;

                    tracks.numTracks++;
                    break;
                }
            }
        }
    }
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


void TrackFileMgr::hungarianAssociate(  DetList &dets,
                                        TrackFile &tracks,
                                        vector<int> &state,
                                        int DET_SIZE,
                                        int TRACK_SIZE
                                        )
{
    cout << "Hungarian: " << endl;
    int TERMINATE;

    if (DET_SIZE <= TRACK_SIZE)
    {
        TERMINATE = DET_SIZE;
    }
    else
    {
        TERMINATE = TRACK_SIZE;
    }
    int assoc               = 0;
    vector<int>             zeros_row(DET_SIZE, 0);
    vector<int>             zeros_col(TRACK_SIZE, 0);
    vector<vector<double>>  cost_matrix(STATE_MAX, vector<double>(STATE_MAX, 0.0));
    vector<vector<double>>  OG_cost_matrix(STATE_MAX, vector<double>(STATE_MAX, 0.0));
    vector<vector<int>>     double_covered(STATE_MAX, vector<int>(STATE_MAX, 0.0));
    vector<bool>            rowCovered(TRACK_SIZE,false);
    vector<bool>            colCovered(DET_SIZE, false);
    vector<int>             zerosRow(TRACK_SIZE, 0);
    vector<int>             zerosCol(DET_SIZE, 0);
    vector<double>          sortArray(DET_SIZE*TRACK_SIZE, 1000000.0);
    vector<int>             idxSort(DET_SIZE*TRACK_SIZE, 0);
    vector<int>             idxRow(TRACK_SIZE, 0);
    vector<int>             idxCol(DET_SIZE, 0);
    double                  smallestVal;

    // GENERATE COST MATRIX
    for (int trk = 0; trk < TRACK_SIZE; trk++)
    {
        for (int det = 0; det < DET_SIZE; det++)
        {
            cost_matrix[trk][det] = euclidean(trk, tracks, det, dets);
        }
    }

    OG_cost_matrix = cost_matrix;
    // print_matrix(cost_matrix, DET_SIZE, TRACK_SIZE);
    //  std::cout << endl;
    // COL reduction
    col_reduce(cost_matrix, DET_SIZE, TRACK_SIZE);
    //  std::cout << endl;
    // print_matrix(cost_matrix, DET_SIZE, TRACK_SIZE);
    // ROW reduction
    row_reduce(cost_matrix, DET_SIZE, TRACK_SIZE);
    //  std::cout << endl;
    // print_matrix(cost_matrix, DET_SIZE, TRACK_SIZE);
    //  std::cout << endl;

     // Track the rows and cols with the largest number of zeros
    for (int track = 0; track < TRACK_SIZE; track++)
    {
        for (int det = 0; det < DET_SIZE; det++)
        {
            if(cost_matrix[track][det] == 0 )
            {
                zerosRow[track]++;
                zerosCol[det]++;
            }
        }
    }   

    // cout << "zeros" << endl;
    // print_state(zerosRow);


    int lines = draw_lines( zerosRow, zerosCol, 
                            rowCovered, colCovered,
                            DET_SIZE, TRACK_SIZE, 
                            cost_matrix
                            );

    // print_state(rowCovered);
    // print_state(colCovered);
    //  std::cout << "A: Lines: " << lines << endl;
    // int iterations = 0;

    while (lines < TERMINATE)
    {   

        // // Add uncovered numbers to array
        int offset = TRACK_SIZE;
        for (int track = 0; track < TRACK_SIZE; track++)
        {  
            for (int det = 0; det < DET_SIZE; det++)
            {   
                if ( !rowCovered[track] && !colCovered[det])
                {
                    sortArray[track*offset + det] = cost_matrix[track][det];
                }
            }
        }   

        // //  std::cout << "B: " <<  endl;
        // // print_state(sortArray);

        // // Sort and find the smallest remaining uncovered number
        bubbleSortSmall(sortArray, idxSort);
        smallestVal = sortArray[0];
        //  std::cout << "C: Smalles Val: " << smallestVal << endl;

        for (int track = 0; track < TRACK_SIZE; track++)
        {  
            for (int det = 0; det < DET_SIZE; det++)
            {   
                if ( !rowCovered[track] && !colCovered[det])
                {
                    cost_matrix[track][det] = cost_matrix[track][det] - smallestVal;
                }
            }
        }   
        //  std::cout << "D: " << endl;
        // print_matrix(cost_matrix, DET_SIZE, TRACK_SIZE);

        // Add the smallest value to numbers covered by two lines
        // First discover those numbers
        for (int track = 0; track < TRACK_SIZE; track++)
        {  
            for (int det = 0; det < DET_SIZE; det++)
            {   
                if ( rowCovered[track] && colCovered[det])
                {
                    cost_matrix[track][det] = cost_matrix[track][det] + smallestVal;
                }
            }
        }   

        //  std::cout << "E: " << endl;
        // print_matrix(cost_matrix, 3);

        // Relocate the zeros
        for (int det = 0; det < DET_SIZE; det++)
        {
            zerosCol[det] = 0;
        }
        for (int track = 0; track < TRACK_SIZE; track++)
        {
            zerosRow[track] = 0;
        }
        for (int track = 0; track < TRACK_SIZE; track++)
        {
            for (int det = 0; det < DET_SIZE; det++)
            {
                if(cost_matrix[track][det] == 0 )
                {
                    zerosRow[track]++;
                    zerosCol[det]++;
                }
            }
        }   
        //  std::cout << "ZEROS CHECK: " << endl;
        // print_state(zerosRow);
        // print_state(zerosCol);


        lines = draw_lines( zerosRow, zerosCol, 
                            rowCovered, colCovered,
                            DET_SIZE, TRACK_SIZE, 
                            cost_matrix
                            );
        //  std::cout << "F: Lines: " << lines << endl;
        // iterations++;

    }

    // std::cout << endl;
    // print_state(sortArray);
    // print_matrix(cost_matrix, 3);

    // NOW EXTRACT THE ASSIGNMENTS 
    // Reinitialize the covered rows
    for (int det = 0; det < DET_SIZE; det++)
    {
        colCovered[det] = false;
    }
    for (int track = 0; track < TRACK_SIZE; track++)
    {
        rowCovered[track] = false;
    }

    for (int track = 0; track < TRACK_SIZE; track++)
    {
        for (int det = 0; det < DET_SIZE; det++)
        {
            if(cost_matrix[track][det] == 0 && !rowCovered[track] && !colCovered[det])
            {
                state[det] = track;
                rowCovered[track]   = true;
                colCovered[det]     = true;
            }
        }
    }   

    // Label detections as correlated or not
    for (int det = 0; det < DET_SIZE; det++)
    {
        if(state[det] != -1)
        {
            dets.detList[det].correlated = true;
            dets.detList[det].corrTrack = state[det];
        }
        else
        {
            dets.detList[det].correlated = false;
            dets.detList[det].corrTrack = state[det];
        }
    }
    // Label Tracks as correlated or not
    for (int det = 0; det < DET_SIZE; det++)
    {
        if (state[det] != -1)
        {
            tracks.trackFiles[state[det]].corrDet = det;
        }
    }

    // Print the cost
    double cost = 0.0;

    for (int det = 0; det < STATE_MAX; det++)
    {
        if(state[det] != -1)
        {
            cost = cost + OG_cost_matrix[state[det]][det];
        }
    }

    cout << "Cost: " << cost << endl;

}

void TrackFileMgr::auctionAssociate(DetList &dets,
                                    TrackFile     &tracks,
                                    vector<int>       &state,
                                    int DET_SIZE,
                                    int TRACK_SIZE
                                    )
{
    cout << "Auction: " << endl;
    bool DONE = false;
    int FRONT   = 0;
    int EMPTY = -1;
    int LOWEST = 0;
    int SEC_LOWEST = 1;
    int lowest;
    int secLowest;
    int assoc = 0;
    int iterations = 1;
    double epsilon = 0.2;

    vector<vector<double>>  cost_matrix(TRACK_SIZE, vector<double>(DET_SIZE, -1.0));
    vector<vector<double>>  OG_cost_matrix(STATE_MAX, vector<double>(STATE_MAX, -1.0));
    vector<int>             indexes(DET_SIZE);
    vector<double>          detCost(DET_SIZE, 0.0);
    vector<double>          dual_cost(DET_SIZE, 0.0);

    for (int track = 0; track < TRACK_SIZE; track++)
    {
        for (int det = 0; det < DET_SIZE; det++)
        {
            cost_matrix[track][det] = euclidean(track, tracks, det, dets);
            // cout << cost_matrix[track][det] << endl;
        }
    }
    OG_cost_matrix = cost_matrix;

    if (TRACK_SIZE > 0)
    {
        vector<int> queue(TRACK_SIZE, -1);
        for (int trk = 0; trk < TRACK_SIZE; trk++)
        {
            queue[trk] = trk;
        }
        DONE = false;


        while((queue[FRONT] != EMPTY)  && !DONE)
        {
            // Re-initialize indexes
            for (int i = 0; i < DET_SIZE; i++)
            {
                indexes[i] = i;
            }

            int trkConsidered = pop_Q(queue);
            // std::cout << "trk in ?: " << trkConsidered << enstatedl;
            // std::cout << "Que:" << endl;
            // print_state(queue);

            

            for (int det = 0; det < DET_SIZE; det++)
            {
                detCost[det] = cost_matrix[trkConsidered][det] - dual_cost[det];
            }
            // std::cout << "Det Costs:" << endl;
            // print_state(detCost);

            bubbleSortSmall(detCost, indexes);

            lowest = indexes[LOWEST];
            secLowest = indexes[SEC_LOWEST];

            if (state[lowest] == -1)
            {
                state[lowest] = trkConsidered;
            }
            else if (state[lowest] != -1)
            {
                push_Q(queue, state[lowest]);
                state[lowest] = trkConsidered;
            }

            
            dual_cost[lowest] = dual_cost[lowest] + (detCost[lowest] - detCost[secLowest]) - epsilon;
            // std::cout << "dual_cost modified: " << dual_cost[lowest] << endl;
            assoc = 0;
            for (int det = 0; det < DET_SIZE; det++)
            {
                if (state[det] != -1)
                {
                    assoc++;
                }
            }

            // std::cout << "iteration: " << iterations++ << endl;
            // std::cout << "STATE:" << endl;
            // print_state(state);
            // std::cout << "assocociations: " << assoc << endl;
            if (assoc == DET_SIZE  || assoc == TRACK_SIZE)
            {
                DONE = true;
            }
            else
            {
                DONE = false;
            }

            // if (iterations == 10)DONE = true;

            std::cout << " " << endl;
        }
    }

    // print_state(state);

    // print unassociated dets
    // std::cout << "Unassociated detections: " << endl;
    // for (int det = 0; det < DET_SIZE; det++)
    // {
    //     if(state[det] == -1)
    //     {
    //         std::cout << det << endl;
    //     }
    // }

    // Label detections as correlated or not
    for (int det = 0; det < DET_SIZE; det++)
    {
        if(state[det] != -1)
        {
            dets.detList[det].correlated = true;
            dets.detList[det].corrTrack = state[det];
        }
        else
        {
            dets.detList[det].correlated = false;
            dets.detList[det].corrTrack = state[det];
        }
    }

    // Label Tracks as correlated or not
    for (int det = 0; det < DET_SIZE; det++)
    {
        if (state[det] != -1)
        {
            tracks.trackFiles[state[det]].corrDet = det;
        }
    }

    double cost = 0.0;
    for (int det = 0; det < STATE_MAX; det++)
    {
        if (state[det] != -1)
        {
            cost = cost + OG_cost_matrix[state[det]][det];
        }
    }

    std::cout << "Cost: " << cost << endl;



}