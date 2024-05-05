#include <iostream>
#include <vector>
#include <random>
#include <typeinfo>
#include <fstream>
#include "TrkUtility.hpp"


using namespace std;

int main()
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
    
    int DET_SIZE        =   idx;
    int TRACK_SIZE      =   7;
    int FRONT           =   0;
    int EMPTY           =   -1;
    int LOWEST          =   0;
    int SEC_LOWEST      =   1;
    int lowest;
    int secLowest;
    int assoc = 0;
    bool DONE = false;
    int iterations = 1;
    double epsilon = 0.2;
    vector<int>             state(STATE_MAX, -1);
    vector<vector<double>>  cost_matrix(TRACK_SIZE, vector<double>(DET_SIZE, -1.0));
    vector<vector<double>>  OG_cost_matrix(STATE_MAX, vector<double>(STATE_MAX, -1.0));
    vector<Detection>       dets(DET_SIZE);
    vector<Track>           tracks(TRACK_SIZE);
    vector<double>          dual_cost(DET_SIZE, 0.0);
    vector<int>             indexes(DET_SIZE);
    vector<double>          detCost(DET_SIZE, 0.0);

    // Assign DETECTIONS from input file
    for (int det = 0; det < DET_SIZE; det++)
    {
        dets[det].x = xHold[det];
        dets[det].y = yHold[det];

        std::cout << dets[det].x << " " << dets[det].y << endl;
    }
    
    tracks[0].x = 105; tracks[0].y = 100;
    tracks[1].x = 205; tracks[1].y = 200;
    tracks[2].x = 300; tracks[2].y = 305;
    tracks[3].x = 75;   tracks[3].y = 110;
    tracks[4].x = 456;  tracks[4].y = 234;
    tracks[5].x = 234;  tracks[5].y = 23;
    tracks[6].x = 35;   tracks[6].y = 274;

    for (int track = 0; track < TRACK_SIZE; track++)
    {
        for (int det = 0; det < DET_SIZE; det++)
        {
            cost_matrix[track][det] = euclidean(det, track, dets, tracks);
            // cout << cost_matrix[track][det] << endl;
        }
    }

    OG_cost_matrix = cost_matrix;

    // print_obj(dets); 
    // print_state(state);
    // vector<vector<double>>  test(3, vector<double>(2, -1.0));
    print_matrix(cost_matrix, DET_SIZE, TRACK_SIZE);

//     std::cout << "Verifying matrix contents:\n";
// for (const auto &row : cost_matrix) {
//     for (double val : row) {
//         std::cout << val << " ";
//     }
//     std::cout << "\n";
// }

    // Fill the Q
    vector<int> queue(DET_SIZE, -1);
    for (int trk = 0; trk < TRACK_SIZE; trk++)
    {
        queue[trk] = trk;
    }

    // print_state(detCost);
    DONE = false;

    while((queue[FRONT] != EMPTY)  && !DONE)
    {
        // Re-initialize indexes
        for (int i = 0; i < DET_SIZE; i++)
        {
            indexes[i] = i;
        }

        int trkConsidered = pop_Q(queue);
        std::cout << "trk in ?: " << trkConsidered << endl;
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
        std::cout << "dual_cost modified: " << dual_cost[lowest] << endl;
        assoc = 0;
        for (int det = 0; det < DET_SIZE; det++)
        {
            if (state[det] != -1)
            {
                assoc++;
            }
        }

        std::cout << "iteration: " << iterations++ << endl;
        std::cout << "STATE:" << endl;
        print_state(state);
        std::cout << "assocociations: " << assoc << endl;
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

    // print_state(state);

    // print unassociated dets
    std::cout << "Unassociated detections: " << endl;
    for (int det = 0; det < DET_SIZE; det++)
    {
        if(state[det] == -1)
        {
            std::cout << det << endl;
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

    return 0;
}