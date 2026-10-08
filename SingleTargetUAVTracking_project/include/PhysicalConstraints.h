/**
 * @file PhysicalConstraints.h
 * @brief Handles the physical aspects in the STUT problem such as kinematic movement prediction.

 * */ 

#pragma once

#include "UAVTracking_parameters.h"
#include "MatrixCalculations.h"
#include <random>
#include <string>

template <typename T>
using Matrix = MatrixObject<T>;

template <typename T>
class PhysicalConstraints{

private:

MatrixCalculations<T> matcalc;

public:

//Generates a velocity vector for treating the UAV movement 
Matrix<T> getVelocity(const size_t& vel_seed, const T& max_velocity)const{
if(max_velocity < 0){
  throw std::invalid_argument("The radar system couldn't be constructed. Reason: Negative maximal velocity."); 
}
Matrix<T> vel_vec(3,1);
std::uniform_real_distribution<T> unit_g(-1,1);
std::uniform_real_distribution<T> u_g(0,max_velocity);
std::mt19937 gen(vel_seed); 

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

//Line search approach method by a first order kinematic
//model x_{k+1} = x_{k} + h*v_{k} for a step-size h and velocity vector v 
//that decides if the UAV should take a reverse movement or not in order to stay within the radar
bool MovementDecision(const UAVTracking_parameters& uavParams,const Matrix<T>& uav, const Matrix<T>& vel_vec)const{

double A_term = matcalc.square(uavParams.least_square.h)*matcalc.square(matcalc.L2Norm(vel_vec));
double B_term = 2*uavParams.least_square.h*matcalc.dot_product(uav,vel_vec);
double C_term = matcalc.square(matcalc.L2Norm(uav)) - uavParams.radii.uav_guess_radius;


double discriminant = -4*A_term*C_term + matcalc.square(B_term);
// If the discriminant is negative, then the UAV takes a reversed movement
if (discriminant < 0){
   return false;
}

double t = (1/(2*A_term))*(-B_term + std::sqrt(discriminant));
//Only move forward if t belongs to [0,1], otherwise reverse the movement
if(t <= 1 && t >= 0){
   return true;
}
return false;

}

// A method that performs the kinematic movement prediction
void predictKinematicMovement(const UAVTracking_parameters& uavParams,Matrix<T>& UAVGuess,Matrix<T>& UAVTruth, Matrix<T>& UAVVelocity,
    Matrix<T>& WindVelocity)const{
   
   Matrix<T> vel_vec(3,1);
   for(size_t i = 0;i < 3;i++){
      vel_vec(i,0) = UAVVelocity(i,0) + WindVelocity(i,0);
   }   
   //This restricts the predicted update for the moving UAV from leaving the radar
   if(!MovementDecision(uavParams,UAVTruth,vel_vec)){
      for(size_t j = 0;j < 3;j++){
          vel_vec(j,0) = -vel_vec(j,0);
         }
      }
      
      for(size_t j = 0;j < 3;j++){
         UAVTruth(j,0) += uavParams.least_square.h*vel_vec(j,0); 
         UAVGuess(j,0) += uavParams.least_square.h*vel_vec(j,0);
       }
}

 };