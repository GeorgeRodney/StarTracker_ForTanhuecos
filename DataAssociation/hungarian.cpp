#include "TrackFileMgr.hpp"

void TrackFileMgr::hungarianAssociate()
{
    // DONT RUN IF THERE IS NOTHING TO DO
    if ((m_tracks.numTracks == 0) || (m_dets.numDets == 0))
    {
        return;
    }

    int DET_SIZE = m_dets.numDets;
    int TRACK_SIZE = m_tracks.numTracks;

    int TERMINATE;

    if (DET_SIZE <= TRACK_SIZE)
    {
        TERMINATE = DET_SIZE;
    }
    else
    {
        TERMINATE = TRACK_SIZE;
    }
    vector<int>             state(STATE_MAX, -1);
    int assoc               = 0;
    vector<int>             zeros_row(DET_SIZE, 0);
    vector<int>             zeros_col(TRACK_SIZE, 0);
    vector<vector<double>>  cost_matrix(TRACK_SIZE, vector<double>(DET_SIZE, 0.0));
    vector<vector<double>>  OG_cost_matrix(TRACK_SIZE, vector<double>(DET_SIZE, 0.0));
    vector<vector<int>>     double_covered(TRACK_SIZE, vector<int>(DET_SIZE, 0.0));
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
            double cost = euclidean(trk, m_tracks, det, m_dets);
            if (cost <= 25) 
            {
                cost_matrix[trk][det] = cost;
            }
            else
            {
                cost_matrix[trk][det] = 1000;
            }
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
            m_dets.detList[det].correlated = true;
            m_dets.detList[det].corrTrack = state[det];
        }
        else
        {
            m_dets.detList[det].correlated = false;
            m_dets.detList[det].corrTrack = state[det];
        }
    }
    // Label Tracks as correlated or not
    for (int det = 0; det < DET_SIZE; det++)
    {
        if (state[det] != -1)
        {
            if (OG_cost_matrix[state[det]][det] < 25)
            {
                m_tracks.trackFiles[state[det]].corrDet = det;
            }
            else
            {
                m_tracks.trackFiles[state[det]].corrDet = -1;
            }
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