//Testing the functionality from UAVTracking.h
#include <gtest/gtest.h>
#include "UAVTracking.h"
#include "UAVTracking_parameters.h"

UAVTracking <double> uav_tracking;

TEST(LeastSquareParams,NegativeParameters){
//Negative seed test
UAVTracking_parameters uavParams; 
uavParams.least_square.eps = -5.0323232;
uavParams.least_square.h = -13.3232;
uavParams.least_square.lambda_tuning_factor = 1.0;
uavParams.least_square.tau = 1.0;
EXPECT_THROW({uav_tracking.UAVTracking_Solver(uavParams);},std::invalid_argument);
}

TEST(Variances,NegativeParameters){
//Negative seed test
UAVTracking_parameters uavParams; 
uavParams.least_square.eps = 5.0323232;
uavParams.least_square.h = 13.3232;
uavParams.least_square.lambda_tuning_factor = 1.0;
uavParams.least_square.tau = 1.0;
uavParams.variance.variance_distances = -31.0;
EXPECT_THROW({uav_tracking.UAVTracking_Solver(uavParams);},std::invalid_argument);
}

TEST(Variances,UnknownMethod){
//Tests if the method is valid
UAVTracking_parameters uavParams; 
uavParams.least_square.eps = 5.0323232;
uavParams.least_square.h = 13.3232;
uavParams.least_square.lambda_tuning_factor = 1.0;
uavParams.least_square.tau = 1.0;
uavParams.variance.variance_distances = 31.0;
uavParams.options.activate_method = "KalmanFilter";
EXPECT_THROW({uav_tracking.UAVTracking_Solver(uavParams);},std::invalid_argument);
}

TEST(UAVTracking,UnstableSolution){
//Unstable solution
UAVTracking_parameters uavParams; 
uavParams.dimension.num_of_landmarks = 100;
uavParams.dimension.num_of_uavs = 2;

uavParams.least_square.eps = 1e-7;
uavParams.least_square.rho_threshold = 1e-7;
uavParams.least_square.h = 1.0;
uavParams.least_square.tau = 1e-7;
uavParams.least_square.lambda_threshold = 1e-2;
uavParams.least_square.lambda_tuning_factor = 2.0;

uavParams.radii.radar_radius = 1e4;
uavParams.radii.uav_guess_radius = 1e5;

uavParams.seeds.initial_velocity_seedvalue = 17;
uavParams.seeds.initial_wind_velocity_seedvalue = 21;
uavParams.seeds.landmark_seedvalue = 23;
uavParams.seeds.uav_guess_seedvalue = 24;
uavParams.seeds.uav_truth_seedvalue = 27;
uavParams.seeds.variance_distances = 29;

uavParams.variance.variance_distances = 1e-1;

uavParams.dynamical_UAV_params.max_uav_velocity = 100.0;
uavParams.dynamical_UAV_params.max_wind_velocity =1e5;

uavParams.options.max_iterations = 50;
uavParams.options.activate_method = "G-N";
uavParams.options.applyWGN = false;
EXPECT_THROW({uav_tracking.UAVTracking_Solver(uavParams);},std::runtime_error);
}
