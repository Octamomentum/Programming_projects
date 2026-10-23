 //The main run file for the UAVTracking project

#include "UAVMission_Solver.h"
#include <fstream>

UAVMission_Solver<double> uav_tracking;

//Choice of parameters by default
UAVTracking_parameters Main_Parameters(){
UAVTracking_parameters uavParams;

uavParams.dimension.num_of_landmarks = 1e2;
uavParams.dimension.num_of_uavs = 2;

uavParams.est.eps = 1e-7;
uavParams.est.eps_0 = 1e-2;
uavParams.est.rho_threshold = 1e-7;
uavParams.est.c_param = 1e-4;
uavParams.est.beta = 0.8;
uavParams.est.tau = 1e-3;
uavParams.est.lambda_threshold = 1e-2;
uavParams.est.lambda_tuning_factor = 2.0;

uavParams.posParams.radar_radius = 1e3;
uavParams.posParams.uav_guess_radius = 1e8;
uavParams.posParams.maximal_offset = 2.0;

uavParams.seeds.initial_velocity_seedvalue = 17;
uavParams.seeds.initial_wind_velocity_seedvalue = 23;
uavParams.seeds.landmark_seedvalue = 27;
uavParams.seeds.uav_guess_seedvalue = 31;
uavParams.seeds.uav_truth_seedvalue = 35;
uavParams.seeds.variance_seedvalue = 41;


uavParams.variance.variance_distances = 1e-2;

uavParams.dynamical_UAV_params.h = 0.1;
uavParams.dynamical_UAV_params.max_uav_velocity = 10.0;
uavParams.dynamical_UAV_params.max_wind_velocity = 8.0;

//Default options
uavParams.options.activate_method = "G-N";
uavParams.options.applyWGN = false;

return uavParams;
}

int main(){

//The order for running the cases for evaluating the methods

//Case 1: G-N, no WGN
UAVTracking_parameters main_params = Main_Parameters();
uav_tracking.UAVTracking_Solver(main_params);

//Case 2: L-M, no WGN
main_params.options.activate_method = "L-M";
uav_tracking.UAVTracking_Solver(main_params);

//Case 3:Kalman, no WGN
main_params.options.activate_method = "Kalman";
uav_tracking.UAVTracking_Solver(main_params);

//Case 4: G-N, WGN
main_params.options.applyWGN = true;
main_params.options.activate_method = "G-N";
uav_tracking.UAVTracking_Solver(main_params);

//Case 5: L-M, WGN
main_params.options.activate_method = "L-M";
uav_tracking.UAVTracking_Solver(main_params);

//Case 6:Kalman, WGN
main_params.options.activate_method = "Kalman";
uav_tracking.UAVTracking_Solver(main_params);

//Write needed parameters for a csv-file to use in Python script
std::ofstream csv ("../output_data/UAVParameters.csv");
csv << "NumOfLandmarks, " << "NumOfUAVs," << "RadarRadius," << "RadialBoundaryGuess," << "VarianceTrueDistances\n";
csv << main_params.dimension.num_of_landmarks << "," << main_params.dimension.num_of_uavs << "," << main_params.posParams.radar_radius << "," <<
 main_params.posParams.uav_guess_radius << "," << main_params.variance.variance_distances << "\n"; 
csv.close();
return 0;
}