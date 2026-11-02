//Testing the functionality from UAVMission_Solver.h
#include <gtest/gtest.h>
#include "UAVMission_Solver.h"

UAVMission_Solver <double> uav_tracking;

TEST(EstimationParams,NonPositiveParameters){
//Non-positive estimation parameters
UAVTracking_parameters uavParams; 
uavParams.est.eps = -5.0323232;
uavParams.dynamical_UAV_params.h = -13.3232;
uavParams.est.lambda_tuning_factor = 1.0;
uavParams.est.tau = 1.0;
uavParams.est.lambda_threshold = 0.0;
uavParams.est.rho_threshold = 3.0;
uavParams.posParams.maximal_offset = -2.0;
uavParams.est.gamma = 0.0;
EXPECT_THROW({uav_tracking.UAVTracking_Solver(uavParams);},std::invalid_argument);
}

TEST(Variances,NonPositiveParameters){
//Non-positive variance parameters
UAVTracking_parameters uavParams; 
uavParams.est.eps = 5.0323232;
uavParams.dynamical_UAV_params.h = 13.3232;
uavParams.est.lambda_tuning_factor = 1.0;
uavParams.est.tau = 1.0;
uavParams.est.lambda_threshold = 1.2;
uavParams.est.rho_threshold = 3.0;
uavParams.posParams.maximal_offset = 2.0;
uavParams.est.gamma = 1e-8;
uavParams.variance.variance_distances = -31.0;
uavParams.dynamical_UAV_params.uav_velocity = -100.0;
uavParams.dynamical_UAV_params.wind_velocity = 1e5;
EXPECT_THROW({uav_tracking.UAVTracking_Solver(uavParams);},std::invalid_argument);
}

TEST(ChoosingMethods,UnknownMethod){
//Tests if the method is invalid
UAVTracking_parameters uavParams; 
uavParams.est.eps = 5.0323232;
uavParams.dynamical_UAV_params.h = 13.3232;
uavParams.est.lambda_tuning_factor = 1.0;
uavParams.est.tau = 1.0;
uavParams.est.lambda_threshold = 1.2;
uavParams.est.rho_threshold = 3.0;
uavParams.posParams.maximal_offset = 2.0;
uavParams.variance.variance_distances = 31.0;
uavParams.est.gamma = 1e-8;
uavParams.dynamical_UAV_params.uav_velocity = 100.0;
uavParams.dynamical_UAV_params.wind_velocity = 1e5;
uavParams.options.activate_method = "L-BFGS";
EXPECT_THROW({uav_tracking.UAVTracking_Solver(uavParams);},std::invalid_argument);
}
