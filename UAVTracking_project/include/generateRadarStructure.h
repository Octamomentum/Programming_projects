/**
 * @file generateRadarStructure.h
 * @brief Construction of the radar environment covering landmark locations, radar anchor points and initial UAV estimates.
 * 
 * A method generating the radar environment is implemented by using pseudo-randomized seeds and generate coordinates in a spherical coordinate system
 * centered at the origin. To generate the environment, a radius is fixed to form the radar domain while other specified
 *  radial boundary is set for the initial UAV estimates. The landmark locations and the radar anchor points are generated within the radar domain while
 *  the initial UAV estimates may be generated either inside or outside the radar environment.
  
 * 
 * */ 

#pragma once
#include "MatrixCalculations.h"
#include <random>
#include <tuple>
#include <cmath>

template <typename T>
using Matrix = MatrixObject<T>;

template <typename T>

class generateRadarStructure{

public:

MatrixCalculations<T> matcalc;

//Method that either generates landmarks, physical radar anchor points or initial UAV estimate
void generateGeometricObject(Matrix<T>& init_matrix,const size_t& object_seed, const T& lb_radius, const T& ub_radius)const{

std::uniform_real_distribution<T> u_gen(-1,1);
std::uniform_real_distribution<T> gen(lb_radius,ub_radius);

std::mt19937 generator(object_seed);

for(size_t i = 0; i < init_matrix.getRows();i++){
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

//Generate the radar domain

auto generateRadar(const UAVTracking_parameters& uavParams) const{

//A check if any of the seeds are negative
const size_t Limit = 1e10;
if(uavParams.seeds.uav_seedvalue >= Limit|| uavParams.seeds.landmark_seedvalue >= Limit
  || uavParams.seeds.radar_anchors_seedvalue >= Limit || uavParams.seeds.variance_seedvalue >= Limit){
  throw std::invalid_argument("The radar system couldn't be constructed. Reason: Negative seed(s)."); 
}

//Ensures that the UAV Tracking problem only is applied for at least three landmarks or more
if(uavParams.dimension.num_of_landmarks < 3 || uavParams.dimension.num_of_landmarks >= Limit){
   throw std::invalid_argument("The radar system couldn't be constructed. Reason: Too few landmarks.");
}

//Checks if the number of UAVs is valid
if(uavParams.dimension.num_of_uavs >= Limit || uavParams.dimension.num_of_uavs == 0){
   throw std::invalid_argument("The radar system couldn't be constructed. Reason: Invalid number of UAVs.");
}

//Controls the radial feasibility

if(uavParams.posParams.radar_radius <= 0 || uavParams.posParams.uav_guess_radius <= 0){
   throw std::invalid_argument("The radar system couldn't be constructed. Reason: Invalid radii parameters.");
}

Matrix<T> LandmarkMatrix(uavParams.dimension.num_of_landmarks,3);
Matrix<T> UAV(uavParams.dimension.num_of_uavs,3);
Matrix<T> RadarAnchors(uavParams.dimension.num_of_uavs,3);

T lb_radius = 0.0;

//Generates the landmark locations and radar anchor points within the radar domain
generateGeometricObject(LandmarkMatrix,uavParams.seeds.landmark_seedvalue,lb_radius,uavParams.posParams.radar_radius);
generateGeometricObject(RadarAnchors,uavParams.seeds.radar_anchors_seedvalue,lb_radius,uavParams.posParams.radar_radius);

 //UAV guesses: inside the radar environment
  if(uavParams.posParams.uav_guess_radius<uavParams.posParams.radar_radius){
    generateGeometricObject(UAV,uavParams.seeds.uav_seedvalue,lb_radius,uavParams.posParams.uav_guess_radius);
    }
  //UAV guesses: outside the radar environment
  else {
     generateGeometricObject(UAV,uavParams.seeds.uav_seedvalue,uavParams.posParams.radar_radius,uavParams.posParams.uav_guess_radius);
  }
return std::make_tuple(LandmarkMatrix,UAV,RadarAnchors);   
}

//Construct the environment consisting of the radar and initial UAV estimates
auto generateGlobalEnvironment(const UAVTracking_parameters& uavParams)const{
auto [LandmarkMatrix, UAV,RadarAnchors] = generateRadar(uavParams);
return std::make_tuple(LandmarkMatrix,UAV,RadarAnchors);
}


};