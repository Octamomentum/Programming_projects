/**
 * @file UAVMission_Solver.h
 * @brief The main method that constructs the radar environment and calls the UAV tracking solver 
 * in order to solve the dynamic data assimilation problem and write results to csv-files
 * */ 

#pragma once
#include <string>
#include "generateRadarStructure.h"
#include "UAVTracking_parameters.h"
#include "UAVTracking.h"

template <typename T>
class UAVMission_Solver{

private:

generateRadarStructure<T> radar;
UAVTracking<T> uav_tracking;

public:

void UAVTracking_Solver(const UAVTracking_parameters& uavParams)const{
//Checks if the necessary parameters are positive
if(uavParams.dynamical_UAV_params.h <= 0 || uavParams.est.eps <= 0 || uavParams.est.tau <= 0 || uavParams.est.lambda_threshold <= 0 ||
    uavParams.est.lambda_tuning_factor <= 0 || uavParams.est.beta <= 0  || uavParams.est.c_param <= 0
   || uavParams.est.rho_threshold <= 0 || uavParams.est.eps_0 <= 0 || uavParams.posParams.maximal_offset < 0){
  throw std::invalid_argument("The mission couldn't start. Reason: Non-positive values for the hyperparameters related to the choosen numerical estimator.");
}
//Checks if the variance is positive
if(uavParams.variance.variance_distances <= 0){
   throw std::invalid_argument("The mission couldn't start. Reason: Non-positive variance relating to true distances relatives to the landmarks.");
}
auto [LandmarkMatrix,UAV,VelocitySeeds,VarianceSeed] = radar.generateGlobalEnvironment(uavParams);
std::cout << "\nWelcome to the UAV Tracking program, UT. The radar system will gather information about the sensor coordinates of " <<
 uavParams.dimension.num_of_uavs << " UAVs.\n";  
uav_tracking.TrackingMission_SensorCoordinates(uavParams,LandmarkMatrix,UAV,VelocitySeeds,VarianceSeed);

}

};

