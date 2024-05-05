#include "TrkUtility.hpp"
#include <random>

using namespace std;

int main ()
{

    //Open the .ods file for writing
    ofstream DETS("detections.csv");

    if (!DETS.is_open()) {
        std::cerr << "Error opening file for writing." << std::endl;
        return 1;
    }
    
    const uint16_t              FPA = 512;
    random_device               rd;             // Non-deterministic seed
    mt19937                     gen(rd());      // Mersenne Twister engine
    uniform_int_distribution<>  dis(0, FPA-1);

    // Calculate the false alarm rate3
    int REAL_DETS       = 1;
    int FALSE_DETS      = 3;
    double false_rate   = FALSE_DETS / (FPA * FPA);
    int DET_SIZE        = REAL_DETS + FALSE_DETS;
    vector<Detection>   dets(DET_SIZE);
    int FRAMES          = 2;
    int TARGET = 0;

    // Originial Target
    dets[TARGET].pos[0] = 100; dets[TARGET].pos[1] = 105;
    vector<double> vel(2);
    vel[0] = 0.5; vel[1] = 1;
    double dt = 1;

    //Write detection file
    // REAL OBJECTS
    for (int frame = 0; frame < FRAMES; frame++)
    {
        for (int j = REAL_DETS; j < DET_SIZE; j++)
        {
            dets[j].pos[0] = dis(gen); dets[j].pos[1] = dis(gen);    
        }
        // Update target position
        // dets[TARGET] = update_pos(dets[TARGET], vel, dt);

        for (int i = 0; i < DET_SIZE; i ++)
        {
            DETS << frame << "\t" << dets[i].pos[0] << "\t" << dets[i].pos[1] << endl;;
        }

        dets[TARGET] = update_pos(dets[TARGET], vel, dt);
    }

    DETS.close();
    return 0;
}