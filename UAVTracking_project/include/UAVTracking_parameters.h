/**
 * @file UAVTracking_parameters.h
 * @brief Relevant parameters for the UAV Tracking problem
 * 
 * Explanation: The parameters are divided in different structures.
 * 
 * dimensionParams: number of landmarks and number of UAVs.
 * estParams: Parameters that are used for the numerical estimation problems (Armijo conditioned line search included).
 * varianceParams: Variance for the added White Gaussian Noise if the option is available.
 * seedParams: Values that are needed to initialize pseudo-randomized seed matrix structures.
 * RadarPosParams: Parameters such as radial boundaries relating to the radar environment and maximal positional error offset measured in meters used
 *  in the method that utilizes Kalman-filtering techniques.
 * UAVMovementParams: Velocity magnitudes for the moving UAVs, and wind disturbances, and also the step-size h.
 * trackingOptions: Options that decides which numerical estimation method to use and if WGN is an option or not.
 * */ 

#pragma once 
#include <string>

struct dimensionParams{
size_t num_of_landmarks = 0;
size_t num_of_uavs = 0;
};

struct estParams{
double eps = 0.0;
double eps_0 = 0.0;
double rho_threshold = 0.0;
double c_param = 0.0;
double beta = 0.0;
double tau = 0.0;
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
size_t variance_seedvalue = 0;
};

struct UAVPosParams{
double radar_radius = 0.0;
double uav_guess_radius = 0.0;
double maximal_offset = 0.0;
};

struct UAVMovementParams{
double h = 0.0;
double max_uav_velocity = 0.0;
double max_wind_velocity = 0.0;
};

struct trackingOptions{
std::string activate_method = "";
bool applyWGN = false;
};


struct UAVTracking_parameters{
dimensionParams dimension;
estParams est;
varianceParams variance;
seedParams seeds;
UAVPosParams posParams;
UAVMovementParams dynamical_UAV_params;
trackingOptions options;
};
