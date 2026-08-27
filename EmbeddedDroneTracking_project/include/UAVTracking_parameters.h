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
size_t change_heading;
double radar_radius;
double uav_truth_radius;
double uav_guess_radius;
double max_velocity;
double h;
double lambda;

//Constructor for the main run file
UAVTracking_parameters(): eps(1e-7), num_of_landmarks(1e4), num_of_uavs(2), landmark_seedvalue(54), uav_guess_seedvalue(23), uav_truth_seedvalue(70),
 initial_velocity_seedvalue(1e3), change_heading(5), radar_radius(1e3), uav_truth_radius(500),
 uav_guess_radius(1500), max_velocity(50), h(1),lambda(1e-2){};

 //Constructor for test_UAVTracking
UAVTracking_parameters(double threshold, size_t landmarks, size_t uavs, size_t lm_seed, size_t uavg_seed, size_t uavr_seed, size_t vel_seed,
size_t change_flight, double radar_rad, double u_t_rad, double u_g_rad, double max_v, double stepsize, double l):
eps(threshold), num_of_landmarks(landmarks), num_of_uavs(uavs), landmark_seedvalue(lm_seed), uav_guess_seedvalue(uavg_seed), uav_truth_seedvalue(uavr_seed),
initial_velocity_seedvalue(vel_seed), change_heading(change_flight),radar_radius(radar_rad), uav_truth_radius(u_t_rad), uav_guess_radius(u_g_rad),
max_velocity(max_v), h(stepsize),lambda(l){};

};
