/**
 * @file generateRadarStructure.h
 * @brief Construction of the radar environment covering landmark locations,
 *  sensor coordinates for assumed UAVs in the field.
 * 
 * - Underlying design: A method generating the radar is implemented by using pseudo-randomized seeds stored in matrix structures. 
 * To generate the environment, a radius is fixed to form the radar perimeter while other specified
 *  spherical boundaries are set for the sensor coordinates.
 * - Radar components: The landmark locations and the ground truth for the sensor coordinates are generated in the range of the 
 * radar following a spherical coordinate system that goes through the origin.
 * - Guessed coordinates: The initial guess for the coordinates are generated in a similar way as the ground truth except they 
 * can be placed either inside or outside the radar.  
 * 
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
Matrix<size_t> vel_uav_seed(uavParams.num_of_uavs,1);
Matrix<size_t> vel_wind_seed(uavParams.num_of_uavs,1);


std::mt19937 gen1(uavParams.landmark_seedvalue);
std::mt19937 gen2(uavParams.uav_truth_seedvalue);
std::mt19937 gen3(uavParams.uav_guess_seedvalue);
std::mt19937 gen4(uavParams.initial_velocity_seedvalue);
std::mt19937 gen5(uavParams.initial_wind_velocity_seedvalue);

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
   vel_uav_seed(i,0) =  u_g(gen4);
}

for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   vel_wind_seed(i,0) =  u_g(gen5);
}

return std::make_tuple(lm_seed,uav_truth_seed,uav_guess_seed,vel_uav_seed,vel_wind_seed);   
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

//Construct the global world consisting of the radar and the guessed sensor coordinates
auto generateGlobalEnvironment(const UAVTracking_parameters& uavParams)const{
if(uavParams.uav_guess_seedvalue < 0 || uavParams.landmark_seedvalue < 0
  || uavParams.initial_velocity_seedvalue < 0 || uavParams.initial_wind_velocity_seedvalue < 0 || uavParams.uav_truth_seedvalue < 0){
  throw std::invalid_argument("The radar system couldn't be constructed. Reason: Negative seed(s)."); 
}
auto [lm_seed,uav_truth_seed,uav_guess_seed,vel_uav_seed,vel_wind_seed] = generateSeeds(uavParams);
auto [LandmarkMatrix, UAVTruth] = generateRadar(uavParams,lm_seed,uav_truth_seed);
Matrix<T> UAVGuess = generateSensorCoordinates(uavParams,uav_guess_seed);
auto UAV = std::make_tuple(UAVTruth,UAVGuess);
auto Velocities = std::make_tuple(vel_uav_seed,vel_wind_seed);
return std::make_tuple(LandmarkMatrix,UAV,Velocities);
}


};