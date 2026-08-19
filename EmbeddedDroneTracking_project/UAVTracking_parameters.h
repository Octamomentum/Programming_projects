//This file is meant to store instances of datasets, one for the run file and one for the
//GTest file

struct UAVTracking_parameters{
double eps;   
int num_of_landmarks;
int num_of_uavs;
int landmark_seedvalue;
int uav_guess_seedvalue;
int uav_ref_seedvalue;
int initial_velocity_seedvalue;
int change_heading;
double radar_radius;
double global_radius;
double max_velocity;
double h;
};

const inline UAVTracking_parameters mainData = {1e-6,10000,2,112,215,811,11,2,1000.0,6000.0,100.0,0.5};
const inline UAVTracking_parameters testData = {1e-6,1000,2,5,7,8,13,2,100.0,120.0,2.0,0.5};

