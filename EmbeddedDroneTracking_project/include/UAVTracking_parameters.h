/**
 * @file UAVTracking_parameters.h
 * @brief Relevant parameters for the STUT problem
 * 
 * Explanation: eps is the used threshold for the G-N approach, num_of_landmarks and num_of_uavs are specific
 * numbers for landmarks and UAVS to be generated in the field, the seedvalue variables are non-negative numbers
 * for using the pseudo-randomized seeds, change_heading is used to set how often
 * the flight heading should change direction for the UAVS for a specific number of iterations, 
 * the radar radius variable dominates the generation of the environment while uav_truth_radius/uav_guess_radius 
 * are fixed radial numbers for where in the environment the sensor coordinates are generated, max_velocity is 
 * a fixed upper bound for how fast the UAVs can travel at most in the field,
 * h is the step-size for the kinematic update, lambda is the fixed regularization parameter to prevent ill-posed behavior in the G-N solver.  
 
 * */ 
struct UAVTracking_parameters{
double eps;   
size_t num_of_landmarks;
size_t num_of_uavs;
size_t landmark_seedvalue;
size_t uav_guess_seedvalue;
size_t uav_truth_seedvalue;
size_t initial_velocity_seedvalue;
size_t initial_wind_velocity_seedvalue;
double radar_radius;
double uav_truth_radius;
double uav_guess_radius;
double max_uav_velocity;
double max_wind_velocity;
double h;
double lambda_tuning_factor;
double constant_drained_battery;
bool activate_LM_method;
double tau;

//Constructor for the main run file
UAVTracking_parameters(): eps(1e-7), num_of_landmarks(1e5), num_of_uavs(2), landmark_seedvalue(28), uav_guess_seedvalue(171), uav_truth_seedvalue(50),
 initial_velocity_seedvalue(190), initial_wind_velocity_seedvalue(99), radar_radius(1e3), uav_truth_radius(1e2), uav_guess_radius(1e5), max_uav_velocity(5.0),
  max_wind_velocity(1e-3), h(0.5), lambda_tuning_factor(2.0), constant_drained_battery(0.7), activate_LM_method(true), tau(1e-5){};

 //Constructor for test_UAVTracking
UAVTracking_parameters(double threshold, size_t landmarks, size_t uavs, size_t lm_seed, size_t uavg_seed, size_t uavr_seed, size_t vel_seed, size_t wind_seed,
double radar_rad, double u_t_rad, double u_g_rad, double max_uav_v, double max_wind_v, double stepsize, double ltf, double const_drained_term, bool useLM, double tau_term):
eps(threshold), num_of_landmarks(landmarks), num_of_uavs(uavs), landmark_seedvalue(lm_seed), uav_guess_seedvalue(uavg_seed), uav_truth_seedvalue(uavr_seed),
initial_velocity_seedvalue(vel_seed), initial_wind_velocity_seedvalue(wind_seed), radar_radius(radar_rad), uav_truth_radius(u_t_rad), uav_guess_radius(u_g_rad),
max_uav_velocity(max_uav_v), max_wind_velocity(max_wind_v), h(stepsize), lambda_tuning_factor(ltf),
 constant_drained_battery(const_drained_term), activate_LM_method(useLM), tau(tau_term){};

};
