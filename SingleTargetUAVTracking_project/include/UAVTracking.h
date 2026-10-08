/**
 * @file UAVTracking.h
 * @brief Radar System Application involving 3D Estimation for isolated UAV states and Dynamic Data Assimilation.
 * 
 * Goal: The main objective is to track and estimate sensor coordinates for moving UAVs in a radar environment
 *  w.r.t. to a data assimilation aspect, where a numerical method strategy is used 
 * to minimize the L2-measures of residual distances relative to the landmarks in the field. The radar system
 *  collects valuable information about L2-measured norms for directional derivative/residuals 
 * and estimated sensor coordinates. Once the strategy has successfully converged, the results are plotted later in Python.
 
 * System setup: 
 * - Initialization parameters: Landmark locations, ground truth knowledge about the sensor coordinates, and initial guess for the sensor coordinates
 * are all determined in a spherical coordinate system using fixed pseudo-random seeds.
 * - Boundaries: The landmarks and the sensor coordinates relative to the ground truth knowledge, are generated in a fixed radar radius range, while
 * the sensor coordinates relative to the initial guess, are able to be generated outside or inside the radar.
 * - Kinematic profile: As the UAVs are moving in the field, the objective navigates the movement for the UAV along local velocity vectors
 *  that changes over states. In this movement, each update is predicted with an explicit step-size h, to then 
 * solve a least square problem using the G-N (or Levenberg-Marquardt) method. The kinematic model is first-order by the formula x_{k+1} = x_{k} + h*v_{k}
 *  for a velocity vector v_{k} at state k.
 * - Noise sources: In this application, an option to add White Gaussian Noise to the true distances of the UAVs relative to landmarks,
 *  is now possible.

 * */ 

#pragma once
#include "MatrixCalculations.h"
#include "generateRadarStructure.h"
#include "LinkedListObject.h"
#include "TrackingParameters.h"
#include "PhysicalConstraints.h"
#include <iostream>
#include <fstream>
#include <tuple>
#include <chrono>
#include <random>

template <typename T>
using List = LinkedListObject<T>;
template <typename T>
using ListOfMatrices = LinkedListObject<Matrix<T>>;

template <typename T>
class UAVTracking{

private:

MatrixCalculations<T> matcalc;
generateRadarStructure<T> radar;
PhysicalConstraints<T> physconst;

public:

//Writes numerical results to csv-files which will be used later for plots in Python.
void fileWriter(const UAVTracking_parameters& uavParams,
   std::tuple<List<T>,List<T>, ListOfMatrices<T>,ListOfMatrices<T>,List<T>>& RadarData, const size_t& ind) const{

List<T> L2error_list = std::get<0>(RadarData);
List<T> distResiduals_list = std::get<1>(RadarData);
ListOfMatrices<T> UAVGuess_list = std::get<2>(RadarData);
ListOfMatrices<T> UAVTruth_list = std::get<3>(RadarData);
List<T> time_list = std::get<4>(RadarData);
std::string method = "";
std::string noiseAnswer = "";

if(uavParams.options.applyWGN){
   noiseAnswer = "WGN";
}
else{
   noiseAnswer = "NoWGN";
}

if(uavParams.options.activate_method == "G-N"){
  method = "GN"; 
}
else if(uavParams.options.activate_method == "L-M"){
  method = "LM"; 
}
else{
  method = "BFGS"; 
}


//Write L2-measures of the directional derivative over states per UAV to csv-file
std::ofstream csv_1 ("output_data/L2Convergence_" + method + "," + noiseAnswer + "_UAV," + std::to_string(ind+1) + ".csv");
csv_1 << "Iterations," << "L2Error\n";
for(size_t j = 0;j < L2error_list.getLength();j++){
   csv_1 << j+1 << "," << L2error_list[j] << "\n";  
}
csv_1.close();

//Write L2-measures of the residual distances over states per UAV to csv-file
std::ofstream csv_2 ("output_data/ResidualErrorConvergence_" + method + "," + noiseAnswer + "_UAV" + std::to_string(ind+1) + ".csv");
csv_2 << "Iterations," << "ResidualError\n";
for(size_t j = 0;j < distResiduals_list.getLength();j++){
   csv_2 << j+1 << "," << distResiduals_list[j] << "\n";
  
}
csv_2.close();

//Write the moving ground truth sensor coordinates over states per UAV to csv-file
std::ofstream csv_3 ("output_data/MovingUAV_" + method + "," + noiseAnswer + std::to_string(ind+1) + ".csv");
csv_3 << "Iterations, UAVTruecoord1, UAVTruecoord2, UAVTruecoord3\n";
for(size_t j = 0;j < UAVTruth_list.getLength();j++){
   auto& uav_t = UAVTruth_list[j];
   csv_3 << j+1 << "," << uav_t(0,0) << "," << uav_t(1,0) << "," << uav_t(2,0) << "\n";
      
}     
csv_3.close();

//Write estimate of the sensor coordinates over states per UAV to csv-file
std::ofstream csv_4 ("output_data/UAVConvergence_" + method + "," + noiseAnswer + "_UAV" + std::to_string(ind+1) + ".csv");
csv_4 << "Iterations, UAVEstcoord1, UAVEstcoord2 , UAVEstcoord3\n";
for(size_t j = 0;j < UAVGuess_list.getLength();j++){
   auto& uav_g = UAVGuess_list[j];
   csv_4 << j+1 << "," << uav_g(0,0) << "," << uav_g(1,0) << "," << uav_g(2,0) << "\n"; 
}
csv_4.close();

//Write elapsed time over states per UAV to csv-file
std::ofstream csv_5 ("output_data/ElapsedTime_" + method + "," + noiseAnswer + "_UAV" + std::to_string(ind+1) + ".csv");
csv_5 << "Iterations, Time\n";
for(size_t j = 0;j < time_list.getLength();j++){
   csv_5 << j+1 << "," << time_list[j] << "\n";
}
csv_5.close();
}

//The radar system logic behind the UAV detection problem
void TrackingMission_SensorCoordinates(const UAVTracking_parameters& uavParams, Matrix<T>& LandmarkMatrix,const std::tuple<Matrix<T>,Matrix<T>>& UAV,
    const std::tuple<Matrix<size_t>,Matrix<size_t>>& velocitySeeds, const Matrix<size_t>& variance_distance_seed)const{

Matrix<size_t> vel_uav_seed = std::get<0>(velocitySeeds);
Matrix<size_t> vel_wind_seed = std::get<1>(velocitySeeds);

Matrix<T> UAVTruth = std::get<0>(UAV);
Matrix<T> UAVGuess = std::get<1>(UAV);

std::cout << "\nMission started. Collecting radar data related to the locations of the UAVs...\n";

 //For each UAV, solve the least square problem over a number of states and collect L2-norm of the solution and residual distances, elapsed time,
//and the corresponding sensor coordinates
T time_sum = 0.0;
for(size_t i = 0;i < uavParams.dimension.num_of_uavs;i++){
  ListOfMatrices<T> UAVTruth_list;
  ListOfMatrices<T> UAVGuess_list;

  //Initialize the ground truth and the guessed sensor coordinates for the UAV.

  Matrix<T> uav_g(3,1);
  for(size_t j = 0;j < 3;j++){
      uav_g(j,0) = UAVGuess(i,j);  
     }
    
   Matrix<T> uav_t(3,1);
      for(size_t j = 0;j < 3;j++){
         uav_t(j,0) = UAVTruth(i,j);  
      }
  List<T> time_list;
  List<T> distResidual_list;
  List<T> L2error_list;
  T lambda = 0.0;
  size_t iter = 0;
  TrackingParameters TrackingParams{LandmarkMatrix,uav_t,uav_g,lambda};
  for(size_t j = 0;j < uavParams.options.max_iterations;j++){
      std::cout << "\nIteration " << j + 1 << "\n";
      auto t_start = std::chrono::steady_clock::now();

      
      size_t vel_uav_val = vel_uav_seed(i,0) + j;
      size_t vel_wind_val = vel_wind_seed(i,0) + j;

      //Generate velocities
      Matrix<T> vel_vec_uav = physconst.getVelocity(vel_uav_val,uavParams.dynamical_UAV_params.max_uav_velocity);
      Matrix<T> vel_vec_wind = physconst.getVelocity(vel_wind_val,uavParams.dynamical_UAV_params.max_wind_velocity);

      //Perform kinematic prediction whether the moving UAV should make a reversed movement in current state or not
      physconst.predictKinematicMovement(uavParams,uav_g,uav_t,vel_vec_uav,vel_vec_wind);
      Matrix<T> vel_vec(3,1);
      for(size_t k = 0;k < 3;k++){
         vel_vec(k,0) = vel_vec_uav(k,0) + vel_vec_wind(k,0);
      }   
      //Solve the least squares problem
      auto ls_output = matcalc.LeastSquares_Solver(uavParams,TrackingParams,j,variance_distance_seed);

      std::cout << "\nNorm, UAV " << i+1 << ": " << std::abs(ls_output.dir_vec_normval) << "\n";
      //Collect L2-measures and sensor coordinates
      L2error_list.addElement(ls_output.dir_vec_normval);
      UAVGuess_list.addElement(uav_g);
      UAVTruth_list.addElement(uav_t);
      distResidual_list.addElement(ls_output.res_normval);

      auto t_end = std::chrono::steady_clock::now();
      std::chrono::duration<T> elapsed_time = t_end-t_start;
      //Collect elapsed time
      time_list.addElement(elapsed_time.count());

      if(ls_output.dir_vec_normval < uavParams.least_square.eps || j == uavParams.options.max_iterations-1){
        iter = j+1;
        break;
      }

      
}
std::cout << "\nNumber of iterations: " << iter << "\n";
for(size_t j = 0;j < time_list.getLength();j++){
   time_sum += time_list[j]; 
   }

//Print final results from the least square solver
std::cout << "\nInitial Sensor coordinates for UAV, Ground Truth " << i+1 << ": \n";
const Matrix<T>& uav_init_t = UAVTruth_list.getFirstElement();
uav_init_t.print(); 

std::cout << "\nInitial Sensor coordinates for UAV, Estimate " << i+1 << ": \n";
const Matrix<T>& uav_init_guess = UAVGuess_list.getFirstElement();
uav_init_guess.print(); 

const Matrix<T>& uav_final_t = UAVTruth_list.getLastElement();
const Matrix<T>& uav_final_guess = UAVGuess_list.getLastElement();

std::cout << "\nSensor coordinates for UAV, Ground Truth " << i+1 << ": \n";
uav_final_t.print(); 
std::cout << "\nSensor coordinates for UAV, Estimate " << i+1 << ": \n";
uav_final_guess.print();

bool badEstimate = true;
size_t counter = 0;
for(size_t i = 0;i < uav_final_t.getRows();i++){
   T res = uav_final_t(i,0) - uav_final_guess(i,0);
   if(std::abs(res) > 1.0){
     throw std::runtime_error("\nInaccurate sensor coordinate estimate. Reason: Too strong wind or badly initial guess for the corresponding UAV.\n");   
   }
   counter++;
}

if(counter == uav_final_guess.getRows()){
  badEstimate = false; 
}

if(!badEstimate){
  std::cout << "\nUAV " << i+1 << " has been detected by the radar system at Iteration " << iter << ". Following estimate:\n"; 
  uav_final_guess.print();
}

//Checks if the converged solution for the UAV is stable.
if(matcalc.L2Norm(uav_final_guess) > uavParams.radii.radar_radius){
   throw std::runtime_error("The convergent solution is unstable. Reason: Inappropriate choice for step-size h and maximal UAV velocity."); 
}

auto RadarData = std::make_tuple(L2error_list,distResidual_list,UAVGuess_list,UAVTruth_list,time_list);
//Write results to csv-files
fileWriter(uavParams,RadarData,i);
}

std::cout << "\n Total elapsed time for seeking " << uavParams.dimension.num_of_uavs << " UAVs: " << time_sum << " seconds.\n";

}

//The main method that constructs the radar environment and calls the UAV tracking solver
//in order to solve the dynamic detection problem and write results to csv-files

void UAVTracking_Solver(const UAVTracking_parameters& uavParams)const{
//Checks if the necessary parameters are non-negative
if(uavParams.least_square.h < 0 || uavParams.least_square.eps < 0 || uavParams.least_square.tau < 0 || uavParams.least_square.lambda_threshold <= 0 ||
    uavParams.least_square.lambda_tuning_factor <= 0){
  throw std::invalid_argument("The mission couldn't start. Reason: Negative values for the least square parameters.");
}
if(uavParams.variance.variance_distances < 0){
   throw std::invalid_argument("The mission couldn't start. Reason: Non-positive variance.");
}
//Checks if the method applied is valid
if(uavParams.options.activate_method != "G-N" && uavParams.options.activate_method != "L-M" && uavParams.options.activate_method != "BFGS"){
   throw std::invalid_argument("Invalid method.");
}

auto [LandmarkMatrix,UAV,VelocitySeeds, VarianceSeed] = radar.generateGlobalEnvironment(uavParams);
std::cout << "\nWelcome to the Single-Target UAV Tracking program, STUT. The radar system will gather information about the sensor coordinates of "
 << uavParams.dimension.num_of_uavs << " UAVs.\n"; 
TrackingMission_SensorCoordinates(uavParams,LandmarkMatrix,UAV,VelocitySeeds, VarianceSeed);
}

};