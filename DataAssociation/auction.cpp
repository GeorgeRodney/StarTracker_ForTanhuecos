#include "TrackFileMgr.hpp"

void TrackFileMgr::auctionAssociate()
{   
    // DONT RUN IF THERE IS NOTHING TO DO
    if ((m_tracks.numTracks == 0) || (m_dets.numDets == 0))
    {
        return;
    }

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

    int DET_SIZE = m_dets.numDets;
    int TRACK_SIZE = m_tracks.numTracks;

    vector<int>             state(STATE_MAX, -1);
    vector<vector<double>>  cost_matrix(TRACK_SIZE, vector<double>(DET_SIZE, -1.0));
    vector<vector<double>>  OG_cost_matrix(STATE_MAX, vector<double>(STATE_MAX, -1.0));
    vector<int>             indexes(DET_SIZE);
    vector<double>          detCost(DET_SIZE, 0.0);
    vector<double>          dual_cost(DET_SIZE, 0.0);

    for (int track = 0; track < TRACK_SIZE; track++)
    {
        for (int det = 0; det < DET_SIZE; det++)
        {
            cost_matrix[track][det] = euclidean(track, m_tracks, det, m_dets);
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


        while((queue[FRONT] != EMPTY)  && (!DONE))
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

    // print unassociated m_dets
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
            m_tracks.trackFiles[state[det]].corrDet = det;
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