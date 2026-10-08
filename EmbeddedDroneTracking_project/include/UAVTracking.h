/**
 * @file UAVTracking.h
 * @brief Radar System Application involving 3D Estimation for dynamic isolated Multi-UAV states and Data Assimilation.
 * 
 * Initializes a radar environment used for predicting the sensor coordinates of a number of moving isolated
 *  UAVs in the field by applying a Gauss-Newton approach that attempts to minimize the L2-measured residual distances relative to the landmarks. Once 
 * the approach has converged, results are asserted and written to csv-files to then plot in Python. 
 *
 * System setup: 
 * - Initialization parameters: Landmark locations, ground truth knowledge about the sensor coordinates, and initial guess for the sensor coordinates
 * are all determined in a spherical coordinate system using fixed pseudo-random seeds.
 * - Boundaries: The landmarks and the sensor coordinates relative to the ground truth knowledge, are generated in a fixed radar radius range, while
 * the sensor coordinates relative to the initial guesses, are able to be generated outside or inside the radar.
 * - Kinematic profile: As the UAVs are moving in the field, the objective navigates the movement for the UAV along local velocity vectors
 *  that changes over time. In this movement, each update is predicted with an explicit step-size h, to then 
 * solve a least square problem using the G-N method. The kinematic model is first-order by the formula x_{k+1} = x_{k} + h*v_{k} for 
 * a velocity vector v_{k} at state k, which can be applied to the steepest descent method.
  
 * */ 

#pragma once
#include "MatrixCalculations.h"
#include "generateRadarStructure.h"
#include "LinkedListObject.h"
#include <iostream>
#include <fstream>
#include <tuple>
#include <chrono>
#include <random>

template <typename T>
using Matrix = MatrixObject<T>;
template <typename T>
using List = LinkedListObject<T>;
template <typename T>
using ListOfMatrices = LinkedListObject<Matrix<T>>;

template <typename T>
class UAVTracking{

private:

MatrixCalculations<T> matcalc;
generateRadarStructure<T> radar;
UAVTracking_parameters uavParams;

public:

//Generates velocity for either the UAV or the wind
Matrix<T> getVelocity(const UAVTracking_parameters& uavParams, const size_t& vel_seed, const T& max_velocity)const{
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

//Steepest descent method by a first order kinematic
//model described earlier that decides if the UAV should take a reverse movement or not in order to stay within the radar
bool MovementDecision(const Matrix<T>& uav, const Matrix<T>& vel_vec)const{

double A_term = matcalc.square(uavParams.h)*matcalc.square(matcalc.L2Norm(vel_vec));
double B_term = 2*uavParams.h*matcalc.dot_product(uav,vel_vec);
double C_term = matcalc.square(matcalc.L2Norm(uav)) - 1;


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

// A method that performs the kinematic prediction update
void predictKinematicMovement(const UAVTracking_parameters& uavParams,Matrix<T>& UAVGuess,Matrix<T>& UAVTruth, Matrix<T>& UAVVelocity,
    Matrix<T>& WindVelocity)const{
   Matrix<T> vel_vec(3,1);
   for(size_t i = 0;i < 3;i++){
      vel_vec(i,0) = UAVVelocity(i,0) + WindVelocity(i,0);
   }   
   //This restricts the predicted update for the sensor coordinates from leaving the radar
   if(!MovementDecision(UAVTruth,vel_vec)){
      for(size_t j = 0;j < 3;j++){
          vel_vec(j,0) = -vel_vec(j,0);
         }
      }
      
      for(size_t j = 0;j < 3;j++){
         UAVTruth(j,0) += uavParams.h*vel_vec(j,0); 
         UAVGuess(j,0) += uavParams.h*vel_vec(j,0);
       }
}

//Writes results from the G-N approach to csv-files which will be used later for plots in Python.
void fileWriter(const std::tuple<List<T>,List<T>, ListOfMatrices<T>,ListOfMatrices<T>,List<T>>& RadarData, const size_t& ind) const{

List<T> L2error_list = std::get<0>(RadarData);
List<T> distResiduals_list = std::get<1>(RadarData);
ListOfMatrices<T> UAVGuess_list = std::get<2>(RadarData);
ListOfMatrices<T> UAVTruth_list = std::get<3>(RadarData);
List<T> time_list = std::get<4>(RadarData);


//Write L2-measures of the direction over iterations per UAV to csv-file
std::ofstream csv_1 ("output_data/L2Convergence_GN_UAV" + std::to_string(ind+1) + ".csv");
csv_1 << "Iterations," << "L2Error\n";
for(size_t j = 0;j < L2error_list.getLength();j++){
   csv_1 << j+1 << "," << L2error_list[j] << "\n";  
}
csv_1.close();

//Write L2-measures of the residual distances over iterations per UAV to csv-file
std::ofstream csv_2 ("output_data/ResidualErrorConvergence_GN_UAV" + std::to_string(ind+1) + ".csv");
csv_2 << "Iterations," << "ResidualError\n";
for(size_t j = 0;j < distResiduals_list.getLength();j++){
   csv_2 << j+1 << "," << distResiduals_list[j] << "\n";
  
}
csv_2.close();

//Write the moving ground truth sensor coordinates over iterations per UAV to csv-file
std::ofstream csv_3 ("output_data/MovingUAV_" + std::to_string(ind+1) + ".csv");
csv_3 << "Iterations, UAVTruecoord1, UAVTruecoord2, UAVTruecoord3\n";
for(size_t j = 0;j < UAVTruth_list.getLength();j++){
   auto& uav_t = UAVTruth_list[j];
   csv_3 << j+1 << "," << uav_t(0,0) << "," << uav_t(1,0) << "," << uav_t(2,0) << "\n";
      
}     
csv_3.close();

//Write estimate of the sensor coordinates over iterations per UAV to csv-file
std::ofstream csv_4 ("output_data/UAVConvergence_GN_UAV" + std::to_string(ind+1) + ".csv");
csv_4 << "Iterations, UAVEstcoord1, UAVEstcoord2 , UAVEstcoord3\n";
for(size_t j = 0;j < UAVGuess_list.getLength();j++){
   auto& uav_g = UAVGuess_list[j];
   csv_4 << j+1 << "," << uav_g(0,0) << "," << uav_g(1,0) << "," << uav_g(2,0) << "\n"; 
}
csv_4.close();

//Write elapsed time over iterations per UAV to csv-file
std::ofstream csv_5 ("output_data/ElapsedTime_GN_UAV" + std::to_string(ind+1) + ".csv");
csv_5 << "Iterations, Time\n";
for(size_t j = 0;j < time_list.getLength();j++){
   csv_5 << j+1 << "," << time_list[j] << "\n";
}
csv_5.close();
}

//The modeled battery timer based on the number of Givens rotations and the velocity magnitude for the sought UAV

void batteryTimer(Matrix<T>& battery, const size_t& uav_ind, const T& velocity_magnitude, const size_t& num_of_rotations)const{

T drained_battery_in_procent = 0.0;
T totNumber_of_rotations = 3*(uavParams.num_of_landmarks-3) + 3;

T velocity_proportion = velocity_magnitude/uavParams.max_uav_velocity;

drained_battery_in_procent += velocity_proportion;

T portion_of_rotations = num_of_rotations/totNumber_of_rotations;

drained_battery_in_procent += portion_of_rotations;

battery(uav_ind,0) -= drained_battery_in_procent;

for(size_t i = 0;i < uavParams.num_of_uavs;i++){
   battery(i,0) -= uavParams.constant_drained_battery;
}

}

//The radar system logic behind the UAV detection problem
void TrackingMission_SensorCoordinates(const Matrix<T>& LandmarkMatrix,const std::tuple<Matrix<T>,Matrix<T>>& UAV,
    const std::tuple<Matrix<size_t>,Matrix<size_t>>& velocitySeeds)const{
Matrix<T> batteryMatrix(uavParams.num_of_uavs,1);
Matrix<size_t> vel_uav_seed = std::get<0>(velocitySeeds);
Matrix<size_t> vel_wind_seed = std::get<1>(velocitySeeds);
for(size_t i = 0;i < uavParams.num_of_uavs;i++){
   batteryMatrix(i,0) = 100.0;
}

Matrix<T> UAVTruth = std::get<0>(UAV);
Matrix<T> UAVGuess = std::get<1>(UAV);

std::cout << "\nMission started. Collecting radar data related to the locations of the UAVs...\n";

bool stillSearching = true;
bool outOfBattery = false;
 //For each UAV, solve the least square problem over a number of iterations and collect L2-norm of the solution and residual distances, elapsed time,
//and the corresponding sensor coordinates
T time_sum = 0.0;
for(size_t i = 0;i < uavParams.num_of_uavs;i++){
  size_t iter = 0;
  ListOfMatrices<T> UAVTruth_list;
  ListOfMatrices<T> UAVGuess_list;

  //Initialize the ground truth and the guessed sensor coordinates for the UAV.

  Matrix<T> uav_g(3,1);
  for(size_t j = 0;j < 3;j++){
      uav_g(j,0) = UAVGuess(j,i);  
     }
    
   Matrix<T> uav_t(3,1);
      for(size_t j = 0;j < 3;j++){
         uav_t(j,0) = UAVTruth(j,i);  
      }
  List<T> time_list;
  List<T> distResidual_list;
  List<T> L2error_list;
  T lam = 0.0;
  T ltp = uavParams.lambda_tuning_factor;
  while(stillSearching){
      std::cout << "\nIteration " << iter + 1 << "\n";
      auto t_start = std::chrono::steady_clock::now();

      size_t vel_uav_val = vel_uav_seed(i,0) + iter;
      size_t vel_wind_val = vel_wind_seed(i,0) + iter;

      //Generate velocities
      Matrix<T> vel_vec_uav = getVelocity(uavParams,vel_uav_val,uavParams.max_uav_velocity);
      Matrix<T> vel_vec_wind = getVelocity(uavParams,vel_wind_val,uavParams.max_wind_velocity);

      //Perform kinematic prediction whether the moving UAV should make a reversed movement in the current flight or not
      predictKinematicMovement(uavParams,uav_g,uav_t,vel_vec_uav,vel_vec_wind);
      Matrix<T> vel_vec(3,1);
      for(size_t j = 0;j < 3;j++){
         vel_vec(j,0) = vel_vec_uav(j,0) + vel_vec_wind(j,0);
      }
      T vel_magnitude = matcalc.L2Norm(vel_vec);
     
      //Solve the least square problem
      auto [uav_g_new,L2Norm_dir_vec,L2Norm_res,num_of_rotations,lambda] = matcalc.LeastSquareSolver_QRDecomposition(LandmarkMatrix,uav_t,uav_g,uavParams.eps, uavParams.tau, lam,
          ltp, uavParams.activate_LM_method,iter);
      uav_g = uav_g_new;
      lam = lambda; 
      //Calculate the L2-norm for the direction vector
      
      std::cout << "\nNorm, UAV " << i+1 << ": " << std::abs(L2Norm_dir_vec) << "\n";
      L2error_list.addElement(L2Norm_dir_vec);
      
      UAVGuess_list.addElement(uav_g);
      UAVTruth_list.addElement(uav_t);
      distResidual_list.addElement(L2Norm_res);
   
      auto t_end = std::chrono::steady_clock::now();
      std::chrono::duration<T> elapsed_time = t_end-t_start;
      //Collect elapsed time
      time_list.addElement(elapsed_time.count());
   
      //Update the flag only if the norm is minimal, otherwise continue the mission
      if(std::abs(L2Norm_dir_vec) < uavParams.eps){
         stillSearching = false;
      }      
      batteryTimer(batteryMatrix,i,vel_magnitude,num_of_rotations);
      for(size_t j = 0;j < uavParams.num_of_uavs;j++){
         std::cout << "\nBattery left for UAV " << j+1 << ": " << batteryMatrix(j,0) << "\n";
         if(batteryMatrix(j,0) < uavParams.eps){
            std::cout << "\nBattery ran out for UAV " << j+1 << "!\n";
            outOfBattery = true;
            break;   
        }
      }
      if(outOfBattery){
         throw std::runtime_error("Radar lost contact with the UAV. Reason: Out of battery.");
        }
      iter += 1;
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

std::cout << "\nSensor coordinates for UAV, Ground Truth " << i+1 << ": \n";
const Matrix<T>& uav_final_t = UAVTruth_list.getLastElement();
uav_final_t.print(); 

std::cout << "\nSensor coordinates for UAV, Estimate " << i+1 << ": \n";
const Matrix<T>& uav_final_guess = UAVGuess_list.getLastElement();
uav_final_guess.print(); 

//Checks if the converged solution for the UAV is stable.
if(matcalc.L2Norm(uav_final_guess) > uavParams.radar_radius){
   throw std::runtime_error("The convergent solution is unstable. Reason: Inappropriate choice for step-size h and maximal velocity."); 
}

auto RadarData = std::make_tuple(L2error_list,distResidual_list,UAVGuess_list,UAVTruth_list,time_list);
fileWriter(RadarData,i);
stillSearching = true; 
}

std::cout << "\n Total elapsed time for finding " << uavParams.num_of_uavs << " UAVs: " << time_sum << " seconds.\n";

}

//The main method that constructs the radar environment and calls the UAV tracking solver
//in order to solve the dynamic detection problem and write results to csv-files

void UAVTracking_Solver(const UAVTracking_parameters& uavParams)const{
//Checks if the necessary parameters are non-negative
if(uavParams.h < 0 || uavParams.eps < 0 || uavParams.tau < 0){
  throw std::invalid_argument("The mission couldn't start. Reason: Negative value(s) for h, epsilion or lambda.");
}
std::cout << "\nWelcome to the Single-Targeted UAV Tracking program, STUT. The radar system will gather information about the sensor coordinates of "
 << uavParams.num_of_uavs << " UAVs.\n"; 
auto [LandmarkMatrix,UAV,VelocitySeeds] = radar.generateGlobalEnvironment(uavParams);
TrackingMission_SensorCoordinates(LandmarkMatrix,UAV,VelocitySeeds);
}

};