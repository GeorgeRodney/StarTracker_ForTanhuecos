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

    float                       sigma_meas = 1;
    mt19937                     meas(rd());
    uniform_real_distribution<>  R(0, sigma_meas);


    // Calculate the false alarm rate3
    int REAL_DETS       = 3;
    int FALSE_DETS      = 6;
    double false_rate   = FALSE_DETS / (FPA * FPA);
    int DET_SIZE        = REAL_DETS + FALSE_DETS;
    vector<Detection>   dets(DET_SIZE);
    int FRAMES          = 100;
    int TARGET = 0;

    // Originial Target
    dets[TARGET].pos[0] = 10; dets[TARGET].pos[1] = 12;
    dets[TARGET+1].pos[0] = 144; dets[TARGET+1].pos[1] = 300;
    dets[TARGET+2].pos[0] = 255; dets[TARGET+2].pos[1] = 255;

    dets[TARGET].truth_id = 0;
    dets[TARGET+1].truth_id = 1;
    dets[TARGET+2].truth_id = 2;

    vector<double> vel1(2);
    vector<double> vel2(2);
    vector<double> vel3(2);

    vel1[0] = 0.5; vel1[1] = 1;
    vel2[0] = 2;   vel2[1] = -0.2;
    vel3[0] = -1;  vel3[1] = 2.2;
    double dt = 1;

    //Write detection file
    // REAL OBJECTS
    for (int frame = 0; frame < FRAMES; frame++)
    {
        for (int j = REAL_DETS; j < DET_SIZE; j++)
        {
            dets[j].pos[0] = dis(gen); 
            dets[j].pos[1] = dis(gen);
            dets[j].truth_id = -1;    
        }
        // Update target position
        // dets[TARGET] = update_pos(dets[TARGET], vel1, dt);

        for (int i = 0; i < DET_SIZE; i ++)
        {
            DETS << frame << "\t" << dets[i].pos[0] + R(meas) 
                            << "\t" << dets[i].pos[1] + R(meas) 
                                << "\t" << dets[i].truth_id << endl;;
        }

        dets[TARGET]   = update_pos(dets[TARGET], vel1, dt);
        dets[TARGET+1] = update_pos(dets[TARGET+1], vel2, dt);
        dets[TARGET+2] = update_pos(dets[TARGET+2], vel3, dt);
    }

    DETS.close();
    return 0;
}