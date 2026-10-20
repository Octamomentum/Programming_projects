/**
 * @file generateRadarStructure.h
 * @brief Construction of the radar environment covering landmark locations, ground truth/guesses sensor coordinates for the moving UAVs.
 * 
 * - Underlying design: A method generating the radar is implemented by using pseudo-randomized seeds stored in matrix structures which includes
 * velocities for UAVs/wind and also optional added White Gaussian Noise to the true distances of the UAVs relative to the landmarks.
 * To generate the environment, a radius is fixed to form the radar perimeter while other specified
 *  spherical boundaries are set for the UAV coordinates.
 * - Radar components: The landmark locations and the ground truth for the UAV
 *  sensor coordinates are generated inside the radar following a spherical coordinate system centered at the origin.
 *  The guesses for the UAV coordinates can be generated inside or outside the radar.
  
 * 
 * */ 

#pragma once
#include "MatrixCalculations.h"
#include <random>
#include <tuple>
#include <math.h>
#include <cmath>

template <typename T>
using Matrix = MatrixObject<T>;

template <typename T>

class generateRadarStructure{

private:

MatrixCalculations<T> matcalc;

public:

//Generate the pseudo-random seed matrices

auto generateSeeds(const UAVTracking_parameters& uavParams)const{
size_t Limit = 1e10;
if(uavParams.seeds.uav_guess_seedvalue >= Limit|| uavParams.seeds.landmark_seedvalue >= Limit
  || uavParams.seeds.initial_velocity_seedvalue >= Limit || uavParams.seeds.initial_wind_velocity_seedvalue >= Limit || 
  uavParams.seeds.uav_truth_seedvalue >= Limit || uavParams.seeds.variance_seedvalue >= Limit){
  throw std::invalid_argument("The radar system couldn't be constructed. Reason: Negative seed(s)."); 
}

//Ensures that the UAV Fleet Tracking problem only is applied for at least three landmarks or more
if(uavParams.dimension.num_of_landmarks < 3 || uavParams.dimension.num_of_landmarks >= Limit){
   throw std::invalid_argument("The radar system couldn't be constructed. Reason: Too few landmarks.");
}

//Checks if the number of UAVs is valid
if(uavParams.dimension.num_of_uavs >= Limit || uavParams.dimension.num_of_uavs == 0){
   throw std::invalid_argument("The radar system couldn't be constructed. Reason: Invalid number of UAVs.");
}

std::uniform_int_distribution<size_t> u_g(0,1e7);

Matrix<size_t> lm_seed(uavParams.dimension.num_of_landmarks,1);
Matrix<size_t> uav_truth_seed(uavParams.dimension.num_of_uavs,1);
Matrix<size_t> uav_guess_seed(uavParams.dimension.num_of_uavs,1);
Matrix<size_t> vel_uav_seed(uavParams.dimension.num_of_uavs,1);
Matrix<size_t> vel_wind_seed(uavParams.dimension.num_of_uavs,1);
Matrix<size_t> variance_seed(uavParams.dimension.num_of_uavs,1);

std::mt19937 gen1(uavParams.seeds.landmark_seedvalue);
std::mt19937 gen2(uavParams.seeds.uav_truth_seedvalue);
std::mt19937 gen3(uavParams.seeds.uav_guess_seedvalue);
std::mt19937 gen4(uavParams.seeds.initial_velocity_seedvalue);
std::mt19937 gen5(uavParams.seeds.initial_wind_velocity_seedvalue);
std::mt19937 gen6(uavParams.seeds.variance_seedvalue);

for(size_t i = 0; i < uavParams.dimension.num_of_landmarks;i++){
   lm_seed(i,0) =  u_g(gen1);
}

for(size_t i = 0; i < uavParams.dimension.num_of_uavs;i++){
   uav_truth_seed(i,0) =  u_g(gen2);
}

for(size_t i = 0; i < uavParams.dimension.num_of_uavs;i++){
   uav_guess_seed(i,0) =  u_g(gen3);
}

for(size_t i = 0; i < uavParams.dimension.num_of_uavs;i++){
   vel_uav_seed(i,0) =  u_g(gen4);
}

for(size_t i = 0; i < uavParams.dimension.num_of_uavs;i++){
   vel_wind_seed(i,0) =  u_g(gen5);
}

for(size_t i = 0; i < uavParams.dimension.num_of_uavs;i++){
   variance_seed(i,0) =  u_g(gen6);
}

return std::make_tuple(lm_seed,uav_truth_seed,uav_guess_seed,vel_uav_seed,vel_wind_seed,variance_seed);   
}

//Generate landmarks or UAV coordinates relative to ground truth/initial guess
void generateGeometricObject(Matrix<T>& init_matrix,const Matrix<size_t>& object_seed, const T& lb_radius, const T& ub_radius)const{

std::uniform_real_distribution<T> u_gen(-1,1);
std::uniform_real_distribution<T> gen(lb_radius,ub_radius);

for(size_t i = 0; i < init_matrix.getRows();i++){
   std::mt19937 generator(object_seed(i,0));
   for(size_t j = 0;j < init_matrix.getCols();j++){
      init_matrix(i,j) = u_gen(generator);
   }
   T vec_length = std::sqrt(matcalc.square(init_matrix(i,0)) + matcalc.square(init_matrix(i,1)) + matcalc.square(init_matrix(i,2)));
   T radius = gen(generator);
   for(size_t j = 0;j < init_matrix.getCols();j++){
      init_matrix(i,j) = radius*(init_matrix(i,j)/vec_length);
   }
   
}

}

//Generate the radar

auto generateRadar(const UAVTracking_parameters& uavParams, const Matrix<size_t>& landmark_seed,const Matrix<size_t>& uav_truth_seed,
   const Matrix<size_t>& uav_guess_seed) const{
//Controls the radial feasibility
if(uavParams.posParams.radar_radius <= 0 || uavParams.posParams.uav_guess_radius <= 0){
   throw std::invalid_argument("The radar system couldn't be constructed. Reason: Invalid radii or radial offsets.");
}

Matrix<T> LandmarkMatrix(uavParams.dimension.num_of_landmarks,3);
Matrix<T> UAVTruth(uavParams.dimension.num_of_uavs,3);
Matrix<T> UAVGuess(uavParams.dimension.num_of_uavs,3); 

T lb_radius = 0.0;

//Generates the landmark locations
generateGeometricObject(LandmarkMatrix,landmark_seed,lb_radius,uavParams.posParams.radar_radius);
generateGeometricObject(UAVTruth,uav_truth_seed,lb_radius,uavParams.posParams.radar_radius);

 //Inside the radar perimeter
  if(uavParams.posParams.uav_guess_radius<uavParams.posParams.radar_radius){
    generateGeometricObject(UAVGuess,uav_guess_seed,lb_radius,uavParams.posParams.uav_guess_radius);
    }
  //Outside the radar perimeter
  else {
     generateGeometricObject(UAVGuess,uav_guess_seed,uavParams.posParams.radar_radius,uavParams.posParams.uav_guess_radius);
  }
return std::make_tuple(LandmarkMatrix,UAVTruth, UAVGuess);   
}

//Construct the environment consisting of the radar and initial guess for the sensor coordinates relative to UAVs 
auto generateGlobalEnvironment(const UAVTracking_parameters& uavParams)const{
auto [lm_seed,uav_truth_seed,uav_guess_seed,vel_uav_seed,vel_wind_seed,VarianceSeed] = generateSeeds(uavParams);
auto [LandmarkMatrix, UAVTruth, UAVGuess] = generateRadar(uavParams,lm_seed,uav_truth_seed,uav_guess_seed);
auto UAV = std::make_tuple(UAVTruth,UAVGuess);
auto VelocitySeeds = std::make_tuple(vel_uav_seed,vel_wind_seed);
return std::make_tuple(LandmarkMatrix,UAV,VelocitySeeds,VarianceSeed);
}


};