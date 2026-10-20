//Testing the functionality from UAVTracking.h
#include <gtest/gtest.h>
#include "UAVMission_Solver.h"

UAVMission_Solver <double> uav_tracking;

TEST(EstimationParams,NonPositiveParameters){
//Non-positive estimation hyperparameters
UAVTracking_parameters uavParams; 
uavParams.est.eps = -5.0323232;
uavParams.est.eps_0 = -1.2;
uavParams.dynamical_UAV_params.h = -13.3232;
uavParams.est.lambda_tuning_factor = 1.0;
uavParams.est.tau = 1.0;
uavParams.est.beta = -0.5;
uavParams.est.c_param = 1.0;
uavParams.est.lambda_threshold = 0.0;
uavParams.est.rho_threshold = 3.0;
uavParams.posParams.maximal_offset = -2.0;
EXPECT_THROW({uav_tracking.UAVTracking_Solver(uavParams);},std::invalid_argument);
}

TEST(Variances,NonPositiveParameters){
//Non-positive variance hyperparameters
UAVTracking_parameters uavParams; 
uavParams.est.eps = 5.0323232;
uavParams.est.eps_0 = 1e-2;
uavParams.dynamical_UAV_params.h = 13.3232;
uavParams.est.lambda_tuning_factor = 1.0;
uavParams.est.tau = 1.0;
uavParams.est.beta = 0.5;
uavParams.est.c_param = 1.0;
uavParams.est.lambda_threshold = 1.2;
uavParams.est.rho_threshold = 3.0;
uavParams.variance.variance_distances = -31.0;
EXPECT_THROW({uav_tracking.UAVTracking_Solver(uavParams);},std::invalid_argument);
}

TEST(ChoosingMethods,UnknownMethod){
//Tests if the method is invalid
UAVTracking_parameters uavParams; 
uavParams.est.eps = 5.0323232;
uavParams.est.eps_0 = 1e-2;
uavParams.dynamical_UAV_params.h = 13.3232;
uavParams.est.lambda_tuning_factor = 1.0;
uavParams.est.tau = 1.0;
uavParams.est.beta = 0.5;
uavParams.est.c_param = 1.0;
uavParams.est.lambda_threshold = 1.2;
uavParams.est.rho_threshold = 3.0;
uavParams.variance.variance_distances = 31.0;
uavParams.options.activate_method = "L-BFGS";
EXPECT_THROW({uav_tracking.UAVTracking_Solver(uavParams);},std::invalid_argument);
}

TEST(UAVTracking,UnstableSolution){
//Unstable solution given a chosen method
UAVTracking_parameters uavParams; 
uavParams.dimension.num_of_landmarks = 100;
uavParams.dimension.num_of_uavs = 2;

uavParams.est.eps = 1e-7;
uavParams.est.eps_0 = 1e-2;
uavParams.dynamical_UAV_params.h = 1e-2;
uavParams.est.lambda_tuning_factor = 2.0;
uavParams.est.tau = 1e-12;
uavParams.est.beta = 0.5;
uavParams.est.c_param = 1e-4;
uavParams.est.lambda_threshold = 1e-2;
uavParams.est.rho_threshold = 1e-7;

uavParams.posParams.radar_radius = 1e3;
uavParams.posParams.uav_guess_radius = 1e4;

uavParams.seeds.initial_velocity_seedvalue = 17;
uavParams.seeds.initial_wind_velocity_seedvalue = 21;
uavParams.seeds.landmark_seedvalue = 23;
uavParams.seeds.uav_guess_seedvalue = 24;
uavParams.seeds.uav_truth_seedvalue = 27;
uavParams.seeds.variance_seedvalue = 12;

uavParams.variance.variance_distances = 1e-1;

uavParams.dynamical_UAV_params.max_uav_velocity = 100.0;
uavParams.dynamical_UAV_params.max_wind_velocity = 1e5;

uavParams.options.activate_method = "G-N";
uavParams.options.applyWGN = false;
EXPECT_THROW({uav_tracking.UAVTracking_Solver(uavParams);},std::runtime_error);
}

TEST(UAVTracking,GuessInsideTheRadarAndUnstableSolution){
//Unstable solution given a chosen method and the guess starts inside the radar
UAVTracking_parameters uavParams; 
uavParams.dimension.num_of_landmarks = 100;
uavParams.dimension.num_of_uavs = 2;

uavParams.est.eps = 1e-7;
uavParams.est.eps_0 = 1e-2;
uavParams.dynamical_UAV_params.h = 1e-2;
uavParams.est.lambda_tuning_factor = 2.0;
uavParams.est.tau = 1e-12;
uavParams.est.beta = 0.5;
uavParams.est.c_param = 1e-4;
uavParams.est.lambda_threshold = 1e-2;
uavParams.est.rho_threshold = 1e-7;

uavParams.posParams.radar_radius = 1e3;
uavParams.posParams.uav_guess_radius = 1e2;

uavParams.seeds.initial_velocity_seedvalue = 17;
uavParams.seeds.initial_wind_velocity_seedvalue = 21;
uavParams.seeds.landmark_seedvalue = 23;
uavParams.seeds.uav_guess_seedvalue = 24;
uavParams.seeds.uav_truth_seedvalue = 27;
uavParams.seeds.variance_seedvalue = 12;

uavParams.variance.variance_distances = 1e-1;

uavParams.dynamical_UAV_params.max_uav_velocity = 100.0;
uavParams.dynamical_UAV_params.max_wind_velocity = 1e5;

uavParams.options.activate_method = "G-N";
uavParams.options.applyWGN = false;
EXPECT_THROW({uav_tracking.UAVTracking_Solver(uavParams);},std::runtime_error);
}

TEST(UAVTracking,WGNAndUnstableSolution){
//Unstable solution given a chosen method and with White Gaussian Noise
UAVTracking_parameters uavParams; 
uavParams.dimension.num_of_landmarks = 100;
uavParams.dimension.num_of_uavs = 2;

uavParams.est.eps = 1e-7;
uavParams.est.eps_0 = 1e-2;
uavParams.dynamical_UAV_params.h = 1e-2;
uavParams.est.lambda_tuning_factor = 2.0;
uavParams.est.tau = 1e-12;
uavParams.est.beta = 0.5;
uavParams.est.c_param = 1e-4;
uavParams.est.lambda_threshold = 1e-2;
uavParams.est.rho_threshold = 1e-7;

uavParams.posParams.radar_radius = 1e3;
uavParams.posParams.uav_guess_radius = 1e4;

uavParams.seeds.initial_velocity_seedvalue = 17;
uavParams.seeds.initial_wind_velocity_seedvalue = 21;
uavParams.seeds.landmark_seedvalue = 23;
uavParams.seeds.uav_guess_seedvalue = 24;
uavParams.seeds.uav_truth_seedvalue = 27;
uavParams.seeds.variance_seedvalue = 12;

uavParams.variance.variance_distances = 1e-1;

uavParams.dynamical_UAV_params.max_uav_velocity = 100.0;
uavParams.dynamical_UAV_params.max_wind_velocity = 1e5;

uavParams.options.activate_method = "L-M";
uavParams.options.applyWGN = true;
EXPECT_THROW({uav_tracking.UAVTracking_Solver(uavParams);},std::runtime_error);
}
