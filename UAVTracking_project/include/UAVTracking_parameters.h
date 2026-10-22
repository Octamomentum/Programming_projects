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

//Number of landmark locations (observations) and the number of UAVs to track
struct dimensionParams{
size_t num_of_landmarks = 0;
size_t num_of_uavs = 0;
};

//Estimation parameters
struct estParams{
//Threshold for the main stop criterion relating to all estimation methods
double eps = 0.0;
//Threshold for the additional stop criterion relating to the absolute error between the L2-norms for the previous and current step solution
double eps_0 = 0.0;
//Threshold for how small the Gain Ratio rho is allowed to get
double rho_threshold = 0.0;
//c_param and beta are parameters related to the Armijo conditioned line search in order to update alpha
double c_param = 0.0;
double beta = 0.0;
//Parameter that acts as a Tikhonov regularization term for Levenberg-Marquardt and the estimation method that use Kalman filtering techniques
double tau = 0.0;
//A threshold for the size of lambda in future iterations, related to the Trust Region approach
double lambda_threshold = 0.0;
//A factor that multiplies lambda if the trial solution is non-acceptable
double lambda_tuning_factor = 0.0;
};

struct varianceParams{
//Variance if WGN is added to the true distances of the UAV positions relative to the landmark locations
double variance_distances = 0.0;

};

//This struct contains parameters for a generated radar environment based on pseudo-randomized seeds
struct seedParams{
size_t landmark_seedvalue = 0;
size_t uav_guess_seedvalue = 0;
size_t uav_truth_seedvalue = 0;
size_t initial_velocity_seedvalue = 0;
size_t initial_wind_velocity_seedvalue = 0;
size_t variance_seedvalue = 0;
};

//This struct contains parameters for the existence of the radar perimeter, every parameter is measured in meters
struct UAVPosParams{
//The radius for the radar perimeter where the landmarks and moving UAVs are generated
double radar_radius = 0.0;
// A radial bound that determines if the initial guesses for the UAV positions are inside or outside the radar perimeter
double uav_guess_radius = 0.0;
//A maximal positional error term known from radar measurements, used for initializing the covariance matrix P 
double maximal_offset = 0.0;
};

//This struct is related to the kinematic movement prediction for the moving UAVs
struct UAVMovementParams{
//Step-size in seconds
double h = 0.0;
//Both parameters below are maximal velocity magnitudes for moving UAVs and wind disturbances, measured in m/s
double max_uav_velocity = 0.0;
double max_wind_velocity = 0.0;
};

//Options for which estimation method to use and if WGN is added to the problem or not
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
