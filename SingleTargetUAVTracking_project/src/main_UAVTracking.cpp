 //The main run file for the UAVTracking project

#include "UAVMission_Solver.h"

UAVMission_Solver<double> uav_tracking;

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
uavParams.posParams.uav_guess_radius = 1e7;
uavParams.posParams.maximal_offset = 2.0;

uavParams.seeds.initial_velocity_seedvalue = 10;
uavParams.seeds.initial_wind_velocity_seedvalue = 24;
uavParams.seeds.landmark_seedvalue = 27;
uavParams.seeds.uav_guess_seedvalue = 29;
uavParams.seeds.uav_truth_seedvalue = 31;
uavParams.seeds.variance_seedvalue = 35;


uavParams.variance.variance_distances = 1e-2;

uavParams.dynamical_UAV_params.h = 0.1;
uavParams.dynamical_UAV_params.max_uav_velocity = 10.0;
uavParams.dynamical_UAV_params.max_wind_velocity = 8.0;

uavParams.options.activate_method = "Kalman";
uavParams.options.applyWGN = true;

return uavParams;
}

int main(){
UAVTracking_parameters main_params = Main_Parameters();
uav_tracking.UAVTracking_Solver(main_params);
return 0;
}