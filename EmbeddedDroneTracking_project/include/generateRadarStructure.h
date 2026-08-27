/**
 * @file generateRadarStructure.h
 * @brief Construction of the radar environment covering landmark locations,
 *  sensor coordinates for assumed UAVs in the field, and their respective initial velocities.
 * 
 * - Underlying design: A method generating the radar is implemented by using pseudo-randomized seeds stored in matrix structures. 
 * To generate the environment, a radial radar is fixed to form the radar perimeter while other specified
 *  radial boundaries are set for the sensor coordinates.
 * - Radar components: The landmark locations and the ground truth for the sensor coordinates are generated in the range of the 
 * radar following a spherical coordinate system that goes through the origin.
 * - Guessed coordinates: The initial guess for the coordinates are generated in a similar way as the ground truth except they 
 * can be placed either inside or outside the radar.  
 * -Initial velocities: For the number of UAVs, their initial velocities are stored in a matrix structure where each column corresponds one UAV.
 * */ 

#include "MatrixCalculations.h"
#include "UAVTracking_parameters.h"
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
Matrix<size_t> lm_seed(uavParams.num_of_landmarks,1);
Matrix<size_t> uav_truth_seed(uavParams.num_of_uavs,1);
Matrix<size_t> uav_guess_seed(uavParams.num_of_uavs,1);
Matrix<size_t> vel_seed(uavParams.num_of_uavs,1);

std::mt19937 gen1(uavParams.landmark_seedvalue);
std::mt19937 gen2(uavParams.uav_truth_seedvalue);
std::mt19937 gen3(uavParams.uav_guess_seedvalue);
std::mt19937 gen4(uavParams.initial_velocity_seedvalue);

std::uniform_int_distribution<size_t> u_g(0,1e7);
for(size_t i = 0; i < uavParams.num_of_landmarks;i++){
   lm_seed(i,0) =  u_g(gen1);
}

for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   uav_truth_seed(i,0) =  u_g(gen2);
}

for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   uav_guess_seed(i,0) =  u_g(gen3);
}

for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   vel_seed(i,0) =  u_g(gen4);
}

return std::make_tuple(lm_seed,uav_truth_seed,uav_guess_seed,vel_seed);   
}

//Generate the radar

auto generateRadar(const UAVTracking_parameters& uavParams, const Matrix<size_t>& landmark_seed,const Matrix<size_t>& uav_truth_seed) const{
//Controls the radial feasibility
if(uavParams.radar_radius <= 0 || uavParams.uav_truth_radius <= 0 || uavParams.radar_radius < uavParams.uav_truth_radius){
   throw std::invalid_argument("The radar system couldn't be constructed. Reason: Invalid radii choices.");
}
//Ensures that the detection problem only is applied for at least three landmarks or more
if(uavParams.num_of_landmarks < 3){
   throw std::invalid_argument("The radar system couldn't be constructed. Reason: Too few landmarks.");
}

Matrix<T> LandmarkMatrix(uavParams.num_of_landmarks,3);
Matrix<T> UAVTruth(3,uavParams.num_of_uavs);

std::uniform_real_distribution<T> u_gen(-1,1);
std::uniform_real_distribution<T> u_lm_radius(0,uavParams.radar_radius);
std::uniform_real_distribution<T> u_uav_t_radius(uavParams.uav_truth_radius,uavParams.radar_radius);

//Generates the landmark locations
for(size_t i = 0; i < uavParams.num_of_landmarks;i++){
   std::mt19937 gen1(landmark_seed(i,0));
   for(size_t j = 0;j < 3;j++){
      LandmarkMatrix(i,j) = u_gen(gen1);
   }
   T vec_length = std::sqrt(matcalc.square(LandmarkMatrix(i,0)) + matcalc.square(LandmarkMatrix(i,1)) + matcalc.square(LandmarkMatrix(i,2)));
   T radius = u_lm_radius(gen1);
   for(size_t j = 0;j < 3;j++){
      LandmarkMatrix(i,j) = radius*(LandmarkMatrix(i,j)/vec_length);
   }
   
}

//Generates the ground truth for the sensor coordinates
for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   std::mt19937 gen2(uav_truth_seed(i,0));
   for(size_t j = 0;j < 3;j++){
      UAVTruth(j,i) = u_gen(gen2);
   }
   T vec_length = std::sqrt(matcalc.square(UAVTruth(0,i)) + matcalc.square(UAVTruth(1,i)) + matcalc.square(UAVTruth(2,i)));
   T radius = u_uav_t_radius(gen2);
   for(size_t j = 0;j < 3;j++){
      UAVTruth(j,i) = radius*(UAVTruth(j,i)/vec_length);
   }
   
}
return std::make_tuple(LandmarkMatrix,UAVTruth);   
}

//Generates the guessed sensor coordinates. The generation is designed to either place the guessed coordinates inside or outside the radar
Matrix<T> generateSensorCoordinates(const UAVTracking_parameters& uavParams,const MatrixObject<size_t>& uav_guess_seed) const{

Matrix<T> UAVGuess(3,uavParams.num_of_uavs);
std::uniform_real_distribution<T> u_gen(-1,1);

if(uavParams.uav_guess_radius <= 0){
   throw std::invalid_argument("The radar system couldn't be constructed. Reason: Invalid guess radius.");
}

//Inside the radar perimeter
if(uavParams.uav_guess_radius<uavParams.radar_radius){
   std::uniform_real_distribution<T> u_uav_g_insideradar(uavParams.uav_guess_radius,uavParams.radar_radius);
   for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   std::mt19937 gen(uav_guess_seed(i,0));
   for(size_t j = 0;j < 3;j++){
       UAVGuess(j,i) = u_gen(gen);
      }
   T vec_length = std::sqrt(matcalc.square(UAVGuess(0,i)) + matcalc.square(UAVGuess(1,i)) + matcalc.square(UAVGuess(2,i)));
   T radius = u_uav_g_insideradar(gen);
   for(size_t j = 0;j < 3;j++){
       UAVGuess(j,i) = radius*(UAVGuess(j,i)/vec_length);
      }
   
}

}
//Outside the radar perimeter
else {
   std::uniform_real_distribution<T> u_uav_g_outsideradar(uavParams.radar_radius,uavParams.uav_guess_radius);
   for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   std::mt19937 gen(uav_guess_seed(i,0));
   for(size_t j = 0;j < 3;j++){
       UAVGuess(j,i) = u_gen(gen);
      }
   T vec_length = std::sqrt(matcalc.square(UAVGuess(0,i)) + matcalc.square(UAVGuess(1,i)) + matcalc.square(UAVGuess(2,i)));
   T radius = u_uav_g_outsideradar(gen);
   for(size_t j = 0;j < 3;j++){
       UAVGuess(j,i) = radius*(UAVGuess(j,i)/vec_length);
      }
   
}
}
  
return UAVGuess;   
}

//Generates initial velocity for a UAV
Matrix<T> getUAVVelocity(const UAVTracking_parameters& uavParams, const size_t& heading_ind, const size_t& vel_seed)const{
if(uavParams.max_velocity < 0){
  throw std::invalid_argument("The radar system couldn't be constructed. Reason: Negative maximal velocity."); 
}
Matrix<T> vel_vec(3,1);
std::uniform_real_distribution<T> unit_g(-1,1);
std::uniform_real_distribution<T> u_g(0,uavParams.max_velocity);
std::mt19937 gen(vel_seed+heading_ind); 

for(size_t j = 0;j < 3;j++){
  vel_vec(j,0) = unit_g(gen);
  } 
T vel = u_g(gen);
T vel_length = matcalc.L2Norm(vel_vec);
for(size_t j = 0;j < 3;j++){
   vel_vec(j,0) = vel*(vel_vec(j,0)/vel_length);
   }

return vel_vec;   
}

//Construct the global world consisting of the radar and the guessed sensor coordinates
auto generateGlobalEnvironment(const UAVTracking_parameters& uavParams)const{
if(uavParams.uav_guess_seedvalue < 0 || uavParams.landmark_seedvalue < 0
  || uavParams.initial_velocity_seedvalue < 0 || uavParams.uav_truth_seedvalue < 0){
  throw std::invalid_argument("The radar system couldn't be constructed. Reason: Negative seed(s)."); 
}
auto [lm_seed,uav_truth_seed,uav_guess_seed,vel_seed] = generateSeeds(uavParams);
auto [LandmarkMatrix, UAVTruth] = generateRadar(uavParams,lm_seed,uav_truth_seed);
Matrix<T> UAVGuess = generateSensorCoordinates(uavParams,uav_guess_seed);
Matrix<T> InitialVelocities(3,uavParams.num_of_uavs);

//Store initial velocities for each UAV column-wise in a matrix structure 
for(size_t i = 0; i<uavParams.num_of_uavs;i++){
   Matrix<T> vel_init = getUAVVelocity(uavParams,0,vel_seed(i,0));
   for(size_t j = 0;j < 3;j++){
      InitialVelocities(j,i) = vel_init(j,0);
   }
}
auto UAV = std::make_tuple(UAVTruth,UAVGuess);
return std::make_tuple(LandmarkMatrix,UAV,InitialVelocities);
}


};