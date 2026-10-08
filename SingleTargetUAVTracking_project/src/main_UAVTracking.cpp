 //The main run file for the UAVTracking project

#include "UAVTracking.h"

UAVTracking<double> uav_tracking;

UAVTracking_parameters Main_Parameters(){
UAVTracking_parameters uavParams;

uavParams.dimension.num_of_landmarks = 1e2;
uavParams.dimension.num_of_uavs = 2;

uavParams.least_square.eps = 1e-7;
uavParams.least_square.rho_threshold = 1e-7;
uavParams.least_square.h = 1.0;
uavParams.least_square.tau = 1e-12;
uavParams.least_square.lambda_threshold = 1e-2;
uavParams.least_square.lambda_tuning_factor = 2.0;

uavParams.radii.radar_radius = 1e4;
uavParams.radii.uav_guess_radius = 1e10;

uavParams.seeds.initial_velocity_seedvalue = 5;
uavParams.seeds.initial_wind_velocity_seedvalue = 13;
uavParams.seeds.landmark_seedvalue = 14;
uavParams.seeds.uav_guess_seedvalue = 17;
uavParams.seeds.uav_truth_seedvalue = 18;
uavParams.seeds.variance_distances = 20;

uavParams.variance.variance_distances = 1e-1;

uavParams.dynamical_UAV_params.max_uav_velocity = 10.0;
uavParams.dynamical_UAV_params.max_wind_velocity = 18.0;

uavParams.options.max_iterations = 50;
uavParams.options.activate_method = "L-M";
uavParams.options.applyWGN = true;

return uavParams;
}

int main(){
UAVTracking_parameters main_params = Main_Parameters();
uav_tracking.UAVTracking_Solver(main_params);
return 0;
}