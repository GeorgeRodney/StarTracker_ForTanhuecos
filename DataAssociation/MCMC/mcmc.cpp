#include <iostream>
#include <vector>
#include <random>
#include <typeinfo>
#include <fstream>
#include "TrkUtility.hpp"


using namespace std;

const int STATE_MAX         = 200;
const int DET_MAX       = STATE_MAX;
const int TRACK_MAX     = STATE_MAX;

double posteriorLikelihood(vector<int> state, vector<Detection> det, vector<Track> track, int cardinality, int numValidDets, double falseAlarmRate, int numTracks)
{
    double assocLike = 0.0;
    double dist = 0.0;

    for (int trk = 0; trk < TRACK_MAX; trk++)
    {
        if (state[trk] != -1)
        {   
            dist = sqrt( pow(track[trk].y - det[state[trk]].y, 2) + pow(track[trk].x - det[state[trk]].x, 2));
            // cout << "Track:" << trk << "  Det:" << state[trk] << "  Dist:" << dist << endl;
            assocLike = assocLike - log(dist);
        }
    }
    assocLike = assocLike   - (numValidDets - cardinality)
                            + (cardinality)
                            + exp(-1 * abs(numValidDets - cardinality));
                            // - (numTracks - cardinality);

    return assocLike;
}   

int main()
{
    uint16_t FPA                        = 512;
    uint16_t numTracks                  = 3;
    uint16_t numDets                    = 3;
    uint16_t numFalseAlarms             = 0;
    random_device                       rd; // Non-deterministic seed
    mt19937                             gen(rd()); // Mersenne Twister engine
    uniform_int_distribution<int>       randTrk(0, numTracks-1);
    uniform_int_distribution<int>       randDet(0, numDets-1);
    uniform_real_distribution<double>   u(0,1);
    int randomTrack = 0;
    int randomDet = 0;
    double wPrime_posterior = 0.0;
    double w_posterior = 0.0;
    double posterior_ratio = 0.0;
    double wLike_posterior = 0.0;
    double falseRate = numFalseAlarms / (FPA*FPA);

    vector<int>     w(TRACK_MAX + 1, -1);
    vector<int>     w_prime(TRACK_MAX + 1, -1);
    vector<int>     w_like(TRACK_MAX + 1, -1);
    vector<int>     w_freq(TRACK_MAX + 1, -1);

    for (int track = 0; track < TRACK_MAX; track++)
    {
        w[track]        = -1;
        w_prime[track]  = -1;
    }
    int cardW           = 0;
    int cardWPrime      = 0;

    vector<int> xHold(DET_MAX, -1);
    vector<int> yHold(DET_MAX, -1);
    vector<int> frameHold(DET_MAX, -1);

    // READ IN DETECTIONS AND TRACKS
    ifstream detections("detections.ods");

    if (!detections.is_open())
    {
        std::cerr << "Error opening file for writing." << std::endl;
        return 1;
    }
    int idx = 0;
    int frame;
    while(detections >> frame)
    {   
        frameHold[idx] = frame;
        detections >> xHold[idx] >> yHold[idx];
        idx++;
    }

    int DET_SIZE        =   idx;
    int TRACK_SIZE      =   3;
    vector<Detection>       dets(DET_SIZE);
    vector<Track>           trackFile(TRACK_SIZE);

    // Assign DETECTIONS from input file
    for (int det = 0; det < DET_SIZE; det++)
    {
        dets[det].x = xHold[det];
        dets[det].y = yHold[det];

        // std::cout << dets[det].x << " " << dets[det].y << endl;
    }
    
    trackFile[0].x = 105;  trackFile[0].y = 100;
    trackFile[1].x = 205;  trackFile[1].y = 200;
    trackFile[2].x = 300;  trackFile[2].y = 305;
    // trackFile[3].x = 75;   trackFile[3].y = 110;

    // Calc number of valid dets
    int validDets = 0;
    double dist = 0.0;
    for (int det = 0; det < numDets; det++)
    {   
        for (int track = 0; track < numTracks; track++)
        {
            dist = sqrt( pow(trackFile[track].y - dets[det].y, 2) + pow(trackFile[track].x - dets[det].x, 2));
            // cout << "Track:" << track << "  Det:" << det << "  Dist:" << dist << endl;
            if (dist < 20.0)
            {
                validDets++;
                break;
            }
        }
    }

    for (int loop = 0; loop < 1000; loop++)
    {
        double U = u(gen);
        if (U < 0.1)
        {
            w_prime = w;
            cardWPrime = cardW;
        }
        else
        {
            // Extract random edge
            randomTrack = randTrk(gen);
            randomDet = randDet(gen);
            bool detAssociated = false;
            bool trackAssociated = false;

            // Find if track is associated 
            if (w[randomTrack] != -1)
            {
                trackAssociated = true;
            }

            // Find if det is associated
            for (int track = 0; track < TRACK_MAX; track++)
            {
                if (w[track] == randomDet)
                {
                    detAssociated = true;
                    break;
                }
            }

            // Perform MOVE selection
            if (trackAssociated && detAssociated && (w[randomTrack] == randomDet)) // DEATH
            {
                w_prime = w;
                cardWPrime = cardW;

                w_prime[randomTrack] = -1;
                cardWPrime--;
            }
            else if (!trackAssociated && !detAssociated)    // BIRTH
            {
                w_prime = w;
                cardWPrime = cardW;

                w_prime[randomTrack] = randomDet;
                cardWPrime++;
            }
            else if (!trackAssociated && detAssociated) // SWAP det already included new track
            {   
                w_prime = w;
                cardWPrime = cardW;

                // Remove det from track
                for (int track = 0; track < TRACK_MAX; track++)
                {
                    if(w[track] == randomDet)
                    {
                        w_prime[track] = -1;
                        break;
                    }
                }
                // Add det to new track
                w_prime[randomTrack] = randomDet;
                
            }
            else if (trackAssociated && !detAssociated)
            {
                w_prime = w;
                cardWPrime = cardW;

                w_prime[randomTrack] = randomDet;
            }
            else
            {
                w_prime = w;
                cardWPrime = cardW;
            }// END MOVE SELECTION
        
            // Calculate the likelihood values
            w_posterior         = posteriorLikelihood(w, dets, trackFile, cardW, validDets, falseRate, numTracks);
            wPrime_posterior    = posteriorLikelihood(w_prime, dets, trackFile, cardWPrime, validDets, falseRate, numTracks);

            // cout << "w_posterior: " << w_posterior << endl;
            // print_state(w);
            // cout << "wPrime_posterior: " << wPrime_posterior << endl;
            // print_state(w_prime);

            posterior_ratio = exp(wPrime_posterior - w_posterior);
            posterior_ratio = min(1.0, posterior_ratio);
            // cout << "posterior_ratio: " << posterior_ratio << endl;

            // Keep track of most frequently occuring state
            for (int track = 0; track < TRACK_MAX; track++)
            {

            }

            // Track Most likely state
            if (wPrime_posterior <= wLike_posterior)
            {
                w_like = w_prime;
                cout << "w_like: " << endl;
                print_state(w_like);

            }
            // Acceptance criteria
            if (u(gen) < posterior_ratio)
            {
                w = w_prime;
                cardW = cardWPrime;
            }
            cout << endl;

        }

    
    }









    return 0;
}