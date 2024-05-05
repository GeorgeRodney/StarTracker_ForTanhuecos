#include <iostream>
#include <fstream>
#include <vector>
#include "TrkUtility.hpp"

using namespace std;

int main ()
{
    ifstream detections("detections.ods");

    if (!detections.is_open())
    {
        std::cerr << "Error opening file for writing." << std::endl;
        return 1;
    }

    const int STATE_MAX = 200;
    int frame;
    const int DET_MAX = STATE_MAX;
    const int TRACK_MAX = STATE_MAX;

    vector<int> xHold(DET_MAX, -1);
    vector<int> yHold(DET_MAX, -1);
    vector<int> frameHold(DET_MAX, -1);
    int idx = 0;

    while(detections >> frame)
    {   
        frameHold[idx] = frame;
        detections >> xHold[idx] >> yHold[idx];
        idx++;
    }
    
    int DET_SIZE            =  idx;
    int TRACK_SIZE          =  7;
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
    vector<int>             state(STATE_MAX, -1);
    vector<int>             zeros_row(DET_SIZE, 0);
    vector<int>             zeros_col(TRACK_SIZE, 0);
    vector<vector<double>>  cost_matrix(STATE_MAX, vector<double>(STATE_MAX, 0.0));
    vector<vector<double>>  OG_cost_matrix(STATE_MAX, vector<double>(STATE_MAX, 0.0));
    vector<vector<int>>     double_covered(STATE_MAX, vector<int>(STATE_MAX, 0.0));
    vector<Detection>       dets(DET_SIZE);
    vector<Track>           tracks(TRACK_SIZE);
    vector<bool>            rowCovered(TRACK_SIZE,false);
    vector<bool>            colCovered(DET_SIZE, false);
    vector<int>             zerosRow(TRACK_SIZE, 0);
    vector<int>             zerosCol(DET_SIZE, 0);
    vector<double>          sortArray(DET_SIZE*TRACK_SIZE, 1000000.0);
    vector<int>             idxSort(DET_SIZE*TRACK_SIZE, 0);
    vector<int>             idxRow(TRACK_SIZE, 0);
    vector<int>             idxCol(DET_SIZE, 0);
    double                  smallestVal;

    // Assign DETECTIONS from input file
    for (int det = 0; det < DET_SIZE; det++)
    {
        dets[det].x = xHold[det];
        dets[det].y = yHold[det];

        //  std::cout << dets[det].x << " " << dets[det].y << endl;
    }

    tracks[0].x = 105; tracks[0].y = 100;
    tracks[1].x = 205; tracks[1].y = 200;
    tracks[2].x = 300; tracks[2].y = 305;
    tracks[3].x = 75;   tracks[3].y = 110;
    tracks[4].x = 456;  tracks[4].y = 234;
    tracks[5].x = 234;  tracks[5].y = 23;
    tracks[6].x = 35;   tracks[6].y = 274;

    for (int trk = 0; trk < TRACK_SIZE; trk++)
    {
        for (int det = 0; det < DET_SIZE; det++)
        {
            cost_matrix[trk][det] = euclidean(det, trk, dets, tracks);
        }
    }

    OG_cost_matrix = cost_matrix;
    print_matrix(cost_matrix, DET_SIZE, TRACK_SIZE);
     std::cout << endl;

    // COL reduction
    col_reduce(cost_matrix, DET_SIZE, TRACK_SIZE);
     std::cout << endl;
    print_matrix(cost_matrix, DET_SIZE, TRACK_SIZE);

    // ROW reduction
    row_reduce(cost_matrix, DET_SIZE, TRACK_SIZE);
     std::cout << endl;
    print_matrix(cost_matrix, DET_SIZE, TRACK_SIZE);
     std::cout << endl;

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
     std::cout << "A: Lines: " << lines << endl;
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
         std::cout << "C: Smalles Val: " << smallestVal << endl;

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
         std::cout << "D: " << endl;
        print_matrix(cost_matrix, DET_SIZE, TRACK_SIZE);

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

    std::cout << endl;
    print_state(state);
    // print_state(zerosCol);

    // print unassociated dets
    std::cout << "Unassociated detections: " << endl;
    for (int det = 0; det < DET_SIZE; det++)
    {
        if(state[det] == -1)
        {
            std::cout << det << endl;
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

    return 0;
}