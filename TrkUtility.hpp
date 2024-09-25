#ifndef TRACK_UTILITY_HPP
#define TRACK_UTILITY_HPP

#include <vector>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <math.h>

using namespace Eigen;

using namespace std;

#define DEGREE 2
#define TRACK_MAX 200
#define STATE_MAX 200
#define DET_MAX   100
#define ACCEL_STD 1
#define STD_POS 1
#define STD_VEL 1
#define STD_MEAS 0.33

enum ValidDet
{
    INVALID_DET = 0,
    VALID_DET
};

enum TrackState{
    CLOSED = 0,
    OPEN,
    CONVERGED
};

class Detection
{
public:
    vector<double> pos;
    bool correlated;
    int corrTrack;
    int valid = INVALID_DET;
    vector<vector<double>> measCov;

    Detection() :   pos(DEGREE, 0.0), 
                    correlated(false), 
                    corrTrack(-1),
                    measCov(2, vector<double>(2, 0.0))
    {
        measCov[0][0] = STD_MEAS*STD_MEAS;
        measCov[1][1] = STD_MEAS*STD_MEAS;
    };

    void clearDet()
    {
        pos[0] = 0;
        pos[1] = 0;
        correlated = false;
        corrTrack = -1;
        valid = INVALID_DET;
    }
};

class Track
{
public:
    vector<double> predVel;
    vector<double> estVel;
    vector<double> predPos;
    vector<double> estPos;
    vector<vector<double>> predCov;
    vector<vector<double>> estCov;
    vector<vector<double>> K;
    vector<vector<double>> H;
    int corrDet = -1;
    TrackState state = CLOSED;
    uint8_t persistance = 0;
    double gate;
    Track() :   predVel(DEGREE,0.0),   
                estVel(DEGREE, 0.0),
                predPos(DEGREE,0.0),
                estPos(DEGREE,0.0),
                predCov(4, vector<double>(4, 0.0)),
                estCov(4, vector<double>(4, 0.0)),
                gate(5.0),
                K(4, vector<double>(2, 0.0)),
                H(2, vector<double>(4, 0.0))
    {
        predCov[0][0] = STD_POS*STD_POS;
        predCov[1][1] = STD_POS*STD_POS;
        predCov[2][2] = STD_VEL*STD_VEL;
        predCov[3][3] = STD_VEL*STD_VEL;

        estCov[0][0] = STD_POS*STD_POS;
        estCov[1][1] = STD_POS*STD_POS;
        estCov[2][2] = STD_VEL*STD_VEL;
        estCov[3][3] = STD_VEL*STD_VEL;

        H[0][0] = 1.0;
        H[1][0] = 1.0;
    };

    // Reset function
    void reset()
    {
        predVel.clear();
        estVel.clear();
        predPos.clear();
        estPos.clear();

        for (int8_t i = 0; i < 4; i++)
        {
            predCov[i].clear();
            estCov[i].clear();
        }

        corrDet = -1;
        state   = CLOSED;
        persistance = 0;
        gate    = 5.0;
    }
    
    void cleanCorrelated()
    {
        corrDet = -1;
    }


};

class DetList
{
    public:
    vector<Detection> detList;
    int numDets = 0;
    DetList(): detList(DET_MAX){};

};

class TrackFile
{
    public:
    vector<Track> trackFiles;
    int numTracks = 0;
    TrackFile(): trackFiles(TRACK_MAX){};

};

inline double euclidean(int trk, TrackFile track, int det, DetList detection)
{
    return sqrt( pow(track.trackFiles[trk].estPos[1] - detection.detList[det].pos[1], 2) + 
                    pow(track.trackFiles[trk].estPos[0] - detection.detList[det].pos[0], 2));
}

inline double statisticalDifferance(int trk, TrackFile track, int det, DetList detection)
{
    vector<vector<double>> S_inv(2, vector<double>(2, 0.0));
    S_inv[0][0] = 1 / (track.trackFiles[trk].predCov[0][0] + detection.detList[det].measCov[0][0]);
    S_inv[1][1] = 1 / (track.trackFiles[trk].predCov[1][1] + detection.detList[det].measCov[1][1]);

    double deltaX = track.trackFiles[trk].predPos[0] - detection.detList[det].pos[0];
    double deltaY = track.trackFiles[trk].predPos[1] - detection.detList[det].pos[1];
    
    double output = deltaX* deltaX * S_inv[0][0] + deltaY * deltaY * S_inv[1][1];
    return output;

}

template <typename T>
void print_state(vector<T> &state)
{
    for (int i = 0; i < state.size(); i++)
    {   
        if (state[i] != -1)
        {
             std::cout << "idx: " << i << ": ";
             std::cout << state[i] <<endl;
        }
    }
}

inline Detection update_pos(Detection det, vector<double> vel, double dt)
{   
    det.pos[0] = det.pos[0] + vel[0] * dt;
    det.pos[1] = det.pos[1] + vel[1] * dt;
    return det;
}

template <typename T>
inline void print_matrix(vector<vector<T>> &matrix, int DET_SIZE, int TRACK_SIZE)
{
    std::cout << "TRACKS" << " X "<< "DETECTIONS" << endl;
    for (int track = 0; track < TRACK_SIZE; track++)
    {
        for (int det = 0; det < DET_SIZE; det++)
        {
            std::cout << setw(2) << matrix[track][det] << ", ";
        }   

         std::cout << endl;
    }
    std::cout << endl;
}

template <typename T>
void print_obj(vector<T> &obj)
{
    std::cout << "Object: " << typeid(T).name() << endl;
    for (int i = 0; i < obj.size(); i++)
    {
        std::cout << "x: " << obj[i].x << " y: " << obj[i].y << endl;
    }
}

template <typename T>
void bubbleSortLarge(vector<T>& arr, vector<int>& idx) {
    int n = arr.size();
    vector<int>     temp(n);
    temp = arr;

    for (int k = 0; k < n; k++)
    {
        idx[k] = k;
    }

    for (int i = 0; i < n - 1; ++i) {
        for (int j = 0; j < n - i - 1; ++j) {
            if (temp[j] < temp[j + 1]) {
                // Swap elements if they are in the wrong order
                swap(temp[j], temp[j + 1]);
                swap(idx[j], idx[j + 1]);
            }
        }
    }
}

template <typename T>
void bubbleSortSmall(vector<T>& arr, vector<int>& idx) {
    int n = arr.size();
    for (int k = 0; k < arr.size(); k++)
    {
        idx[k] = k;
    }

    for (int i = 0; i < n - 1; ++i) {
        for (int j = 0; j < n - i - 1; ++j) {
            if (arr[j] > arr[j + 1]) {
                // Swap elements if they are in the wrong order
                swap(arr[j], arr[j + 1]);
                swap(idx[j], idx[j + 1]);
            }
        }
    }
}

inline void push_Q(vector<int> &Q, int push)
{
    for (int k = Q.size()-2; k >= 0; k--)
    {
        Q[k+1] = Q[k];
    }

    Q[0] = push;
}

inline int pop_Q(vector<int> &Q)
{
    int pop = Q[0];
    for (int k = 0; k < Q.size()-1; k++)
    {
        Q[k] = Q[k+1];
    }
    Q[Q.size()-1] = -1;

    return pop;
}

inline void col_reduce(vector<vector<double>> &costMatrix, int DET_SIZE, int TRACK_SIZE)
{
    for (int det = 0; det < DET_SIZE; det++)
    {
        double minCol = 1000000.0;
        for (int track = 0; track < TRACK_SIZE; track++)
        {
            if (costMatrix[track][det] < minCol)
            {
                minCol = costMatrix[track][det];
            }
        }

        for (int track = 0; track < TRACK_SIZE; track++)
        {
            costMatrix[track][det] = costMatrix[track][det] - minCol;
        }
    }
}

inline void row_reduce(vector<vector<double>> &costMatrix, int DET_SIZE, int TRACK_SIZE)
{
    for (int track = 0; track < TRACK_SIZE; track++)
    {
        double minRow = 100000.0;
        for ( int det = 0; det < DET_SIZE; det++)
        {
            if(costMatrix[track][det] < minRow)
            {
                minRow = costMatrix[track][det];
            }
        }

        for (int det = 0; det < DET_SIZE; det++)
        {
            costMatrix[track][det] = costMatrix[track][det] - minRow;
        }
    }
}

inline int draw_lines( vector<int> &zerosRow, vector<int> &zerosCol, 
                vector<bool> &rowCovered, vector<bool> &colCovered,
                int DET_SIZE, int TRACK_SIZE, 
                vector<vector<double>> cost_matrix
                )
{

    int uncoveredZeros = 1;
    int lines = 0;
    vector<int>             idxCols(DET_SIZE,0);
    vector<int>             idxRows(TRACK_SIZE,0);
    for (int det = 0; det < DET_SIZE; det++)
    {
        colCovered[det] = false;
    }
    for (int track = 0; track < TRACK_SIZE; track++)
    {
        rowCovered[track] = false;
    }

    //  std::cout << "init lines: " << lines <<endl;
    while (uncoveredZeros != 0)
    {   
        //  std::cout << "COST MATRIX CHECK: " << endl;
        //  std::cout <<endl;
        // print_matrix(cost_matrix, 3);

        // print_state(zerosRow);
        bubbleSortLarge(zerosRow, idxRows);
        bubbleSortLarge(zerosCol, idxCols);

        // print_state(zerosRow);

        if (zerosRow[idxRows[0]] >= zerosCol[idxCols[0]])
        {
            rowCovered[idxRows[0]] = true;
            zerosRow[idxRows[0]] = 0;
            
            // Remove the zeros from the col data structure if draw a line through a row
            for (int track = 0; track < TRACK_SIZE; track++)
            {
                for(int det = 0; det < DET_SIZE; det++)
                {
                    if (cost_matrix[track][det] == 0 && idxRows[0] == track)
                    {
                        zerosCol[det] = 0;
                    }
                }
                // cout << " ZEROS Col removal:" << endl;
                // print_state(zerosCol);
            }

            lines++;
            //  std::cout << " Row Covered: "<< idxRows[0] << endl;
        }
        else
        {
            colCovered[idxCols[0]] = true;
            zerosCol[idxCols[0]] = 0;

            // Remove the zeros from the row representation as well
            for (int track = 0; track < TRACK_SIZE; track++)
            {
                for(int det = 0; det < DET_SIZE; det++)
                {
                    if (cost_matrix[track][det] == 0 && idxCols[0] == det)
                    {
                        zerosRow[track] = 0;
                        
                    }
                }

                // cout << " ZEROS Row removal:" << endl;
                // print_state(zerosRow);
            }

            lines++;
            //  std::cout << "Col Covered:" << idxCols[0] << endl;
        }

        // cout << " ZEROS Row removal:" << endl;
        // print_state(zerosRow);
        // cout << " ZEROS Col removal:" << endl;
        // print_state(zerosCol);

        uncoveredZeros = 0;
        // cout << "Rows Covered and then Col Covered" << endl;
        // print_state(rowCovered);
        // print_state(colCovered);
        for (int track = 0; track < TRACK_SIZE; track++)
        {
            for (int det = 0; det < DET_SIZE; det++)
            {
                if(cost_matrix[track][det] == 0 && !rowCovered[track] && !colCovered[det])
                {
                    uncoveredZeros++;
                }
            }
        } 
        // cout << "Uncovered Zeros: " << uncoveredZeros << endl;

        //  std::cout << "Uncovered ZEROS: " << uncoveredZeros << endl;
        //  std::cout << "Lines: " << lines << endl;
        //  std::cout << endl;
    }

    return lines;

}

#endif // TRACK_UTILITY_HPP
