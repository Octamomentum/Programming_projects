/**
 * @file UAVTracking_parameters.h
 * @brief Relevant parameters for the STUT problem
 * 
 * Explanation: The parameters are divided in different structures.
 * 
 * dimensionParams: number of landmarks and number of UAVs.
 * lsParams: Parameters that are used partly for the least square problems while step-length h is used to update the kinematic movement for the UAVs
 * in the radar field.
 * varianceParams: Variances for the added White Gaussian Noise if the option is available.
 * seedParams: Values that are needed to initialize pseudo-randomized seed matrix structures.
 * radiiParams: Different radius boundaries needed to generate the radar environment as it follows a spherical coordinate system centered at the origin.
 * UAVMovementParams: Velocity magnitudes for the moving UAV and wind.
 * trackingOptions: Maximal number of states and options that decides which numerical method to use and if WGN is an option or not.
 * */ 

#pragma once 
#include <string>

struct dimensionParams{
size_t num_of_landmarks = 0;
size_t num_of_uavs = 0;
};

struct lsParams{
double eps = 0.0;
double rho_threshold = 0.0;
double tau = 0.0;
double h = 0.0;
double lambda_threshold = 0.0;
double lambda_tuning_factor = 0.0;
};

struct varianceParams{
double variance_distances = 0.0;
};

struct seedParams{

size_t landmark_seedvalue = 0;
size_t uav_guess_seedvalue = 0;
size_t uav_truth_seedvalue = 0;
size_t initial_velocity_seedvalue = 0;
size_t initial_wind_velocity_seedvalue = 0;
size_t variance_distances = 0;

};

struct radiiParams{
double radar_radius = 0.0;
double uav_guess_radius = 0.0;
};

struct UAVMovementParams{
double max_uav_velocity = 0.0;
double max_wind_velocity = 0.0;
};

struct trackingOptions{
size_t max_iterations = 0;
std::string activate_method = "";
bool applyWGN = false;
};


struct UAVTracking_parameters{
dimensionParams dimension;
lsParams least_square;
varianceParams variance;
seedParams seeds;
radiiParams radii;
UAVMovementParams dynamical_UAV_params;
trackingOptions options;
};
