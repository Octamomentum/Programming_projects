/**
 * @file UAVTracking.h
 * @brief Radar System Application involving 3D Estimation for moving UAVs and Dynamic Data Assimilation.
 * 
 * Goal: The main objective is to track and estimate sensor coordinates relating to moving UAVs in a radar environment
 *  w.r.t. to a dynamic data assimilation aspect, where a numerical estimation method strategy is used 
 * to minimize the L2-measures of residual distances relative to the landmarks in the field. The radar system
 *  collects valuable information about L2-measured norms for directional derivative/residuals 
 * and estimated UAV coordinates. Once the strategy has successfully converged, the results are plotted later in Python.
 
 * System setup: 
 * - Initialization parameters: Landmark locations, ground truth knowledge about the UAV sensor coordinates, and initial guess for the UAV sensor coordinates
 * are all determined in a spherical coordinate system using fixed pseudo-random seeds.
 * - Boundaries: The landmarks and the UAV sensor coordinates relative to the ground truth knowledge, are generated in a fixed radar radius range, while
 * the UAV sensor coordinates relative to the initial guess, are able to be generated outside or inside the radar.
 * - Kinematic profile: As the UAVs are moving in the field, the objective navigates the movement for the UAV along local velocity vectors
 *  that changes over states. In this movement, each update is predicted with an explicit step-size h, to then 
 * solve a least square problem using the G-N (or Levenberg-Marquardt or an approach using Kalman filtering techniques)
 *  method. The kinematic model is first-order by the formula x_{k+1} = x_{k} + h*v_{k}
 *  for a velocity vector v_{k} at state k.
 * - Noise sources: In this application, an option to add White Gaussian Noise to the true distances of the UAVs relative to landmarks,
 *  is now possible. 
 * Kalman filtering: UAVs in this project are mutually independent meaning that the covariance matrix P that describes the uncertainty in
 *  the initial state simply is the diagonal matrix with diagonal elements that is the variance for the maximal UAV positional offset. 

 * */ 

#pragma once
#include "MatrixCalculations.h"
#include "StateEstimator.h"
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
using CollOfValues = LinkedListObject<LinkedListObject<T>>;
template <typename T>
using CollOfMatrices = LinkedListObject<LinkedListObject<Matrix<T>>>;

template <typename T>
class UAVTracking{

private:

MatrixCalculations<T> matcalc;
StateEstimator<T> state_est;
generateRadarStructure<T> radar;
PhysicalConstraints<T> physconst;

public:

//Generate collections of empty lists for the radar application
auto generateCollections(const size_t& num_of_uavs)const{

CollOfMatrices<T> TruthColl;
CollOfMatrices<T> GuessColl;

for(size_t i = 0;i < num_of_uavs;i++){
   ListOfMatrices<T> TruthList;
   ListOfMatrices<T> GuessList;

   TruthColl.addElement(TruthList);
   GuessColl.addElement(GuessList);
}

return std::make_tuple(TruthColl,GuessColl);

}


//Writes numerical results to csv-files which will be used later for plots in Python.
void fileWriter(const UAVTracking_parameters& uavParams, std::tuple<List<T>,List<T>, CollOfMatrices<T>,CollOfMatrices<T>,List<T>>& RadarData) const{
List<T> L2Norm_SolList = std::get<0>(RadarData);
List<T> L2Norm_ResList = std::get<1>(RadarData);
CollOfMatrices<T> UAVTruthColl = std::get<2>(RadarData);
CollOfMatrices<T> UAVGuessColl = std::get<3>(RadarData);
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
  method = "KALMANFILTER"; 
}

size_t num_of_iterations = L2Norm_ResList.getLength();

//Write L2-measures of the directional derivative over states to csv-file
std::ofstream csv_1 ("../output_data/L2Convergence_" + method + "," + noiseAnswer + ".csv");
csv_1 << "Iterations," << "L2Error\n";
for(size_t j = 0;j < num_of_iterations;j++){
   csv_1 << j+1 << "," << L2Norm_SolList[j] << "\n";  
}
csv_1.close();

//Write L2-measures of the residual distances over states to csv-file
std::ofstream csv_2 ("../output_data/ResidualErrorConvergence_" + method + "," + noiseAnswer + ".csv");
csv_2 << "Iterations," << "ResidualError\n";
for(size_t j = 0;j < num_of_iterations;j++){
   csv_2 << j+1 << "," << L2Norm_ResList[j] << "\n";
  
}
csv_2.close();

//Write the moving ground truth UAV sensor coordinates over states per UAV to csv-file
std::ofstream csv_3 ("../output_data/MovingUAV_" + method + "," + noiseAnswer + ".csv");
csv_3 << "Iterations, ";
for(size_t i = 0;i < uavParams.dimension.num_of_uavs;i++){
   csv_3 << "TrueCoord1_UAV" << i << ",TrueCoord2_UAV" << i <<",TrueCoord3_UAV" << i;
   if(i < uavParams.dimension.num_of_uavs-1){
     csv_3 << ","; 
   }
}
csv_3 << "\n";
for(size_t i = 0;i < num_of_iterations;i++){
   csv_3 << i+1 << ",";
for(size_t j = 0;j < uavParams.dimension.num_of_uavs;j++){
   auto& UAVTruthList = UAVTruthColl[j];
   auto& uav_t = UAVTruthList[i];
   csv_3 << uav_t(0,0) << "," << uav_t(1,0) << "," << uav_t(2,0);
   if(j < uavParams.dimension.num_of_uavs-1){
      csv_3 << ",";
   }      
}
csv_3 << "\n";
}     
csv_3.close();

//Write estimate of the UAV sensor coordinates over states per UAV to csv-file
std::ofstream csv_4 ("../output_data/UAVConvergence_" + method + "," + noiseAnswer + ".csv");
csv_4 << "Iterations, ";
for(size_t i = 0;i < uavParams.dimension.num_of_uavs;i++){
   csv_4 << "EstimateCoord1_UAV" << i << ",EstimateCoord2_UAV" << i <<",EstimateCoord3_UAV" << i;
   if(i< uavParams.dimension.num_of_uavs-1){
     csv_4 << ","; 
   }
}
csv_4 << "\n";
for(size_t i = 0;i < num_of_iterations;i++){
   csv_4 << i+1 << ",";
for(size_t j = 0;j < uavParams.dimension.num_of_uavs;j++){
   auto& UAVGuessList = UAVGuessColl[j];
   auto& uav_g = UAVGuessList[i];
   csv_4 << uav_g(0,0) << "," << uav_g(1,0) << "," << uav_g(2,0);
   if(j < uavParams.dimension.num_of_uavs-1){
      csv_4 << ",";
   }      
}
csv_4 << "\n";
}     
csv_4.close();

//Write elapsed time for solving a least square problem over states to csv-file
std::ofstream csv_5 ("../output_data/ElapsedTime_" + method + "," + noiseAnswer + ".csv");
csv_5 << "Iterations, Time\n";
for(size_t j = 0;j < num_of_iterations;j++){
   csv_5 << j+1 << "," << time_list[j] << "\n";
}
csv_5.close();
}


//The radar system logic behind the UAV detection problem
void TrackingMission_SensorCoordinates(const UAVTracking_parameters& uavParams, Matrix<T>& LandmarkMatrix,const std::tuple<Matrix<T>,Matrix<T>>& UAV,
    const std::tuple<Matrix<size_t>,Matrix<size_t>>& velocitySeeds,const Matrix<size_t>& VarianceSeed)const{

Matrix<size_t> vel_uav_seed = std::get<0>(velocitySeeds);
Matrix<size_t> vel_wind_seed = std::get<1>(velocitySeeds);

Matrix<T> UAVTruth = std::get<0>(UAV);
Matrix<T> UAVGuess = std::get<1>(UAV);

std::cout << "\nMission started. Collecting radar data related to the locations of the UAVs...\n";

T time_sum = 0.0;
T abs_norm = 1e5;
T old_norm = 0.0;
T new_norm = 0.0;
List<T> time_list;
List<T> L2_dirvec_list;
List<T> L2_res_list;
auto [TruthColl,GuessColl] = generateCollections(uavParams.dimension.num_of_uavs);

//Initialize the ground truth and guess UAV coordinates for the first state

for(size_t i = 0; i < uavParams.dimension.num_of_uavs;i++){
   ListOfMatrices<T>& TruthList = TruthColl[i];
   Matrix<T> uav_t(3,1); 
   for(size_t j = 0;j < 3;j++){
      uav_t(j,0) = UAVTruth(i,j);
   }
   TruthList.addElement(uav_t);
}
for(size_t i = 0; i < uavParams.dimension.num_of_uavs;i++){
   ListOfMatrices<T>& GuessList = GuessColl[i];
   Matrix<T> uav_g(3,1); 
   for(size_t j = 0;j < 3;j++){
      uav_g(j,0) = UAVGuess(i,j);
   }
   GuessList.addElement(uav_g);
}
T lambda = 0.0;
size_t iter = 0;

//Initialize covariance matrix P for the Kalman filtering approach
Matrix<T> IdentityMatrix_termP = matcalc.getIdentityMatrix(3*uavParams.dimension.num_of_uavs);
//P is treated initially as the diagonal matrix consisting of squared maximal UAV positional error measured in meters, meaning P measures symmetric
//uncertainty in the initial states for the UAVs
Matrix<T> P = IdentityMatrix_termP.scalarMatrix(matcalc.square(uavParams.posParams.maximal_offset));
TrackingParameters TrackingParams{LandmarkMatrix,UAVTruth,UAVGuess,P,lambda};

bool MissionStillContinuing = true;
 //Solve the UAV Tracking problem over a number of states and collect L2-norm of the solution and residual distances, elapsed time,
//and the corresponding UAV sensor coordinates
while(MissionStillContinuing){
    iter++;
      auto varianceParams = std::make_tuple(VarianceSeed,iter);
      std::cout << "\nIteration " << iter << "\n";
      auto t_start = std::chrono::steady_clock::now();

      //Perform kinematic prediction whether the moving UAVs should make a reversed movement in current state or not
      physconst.predictKinematicMovement(uavParams,TrackingParams,iter,vel_uav_seed,vel_wind_seed);
  
      //Solve the numerical estimation problem
      auto est_output = state_est.State_Estimator(uavParams,TrackingParams,varianceParams);

      new_norm = est_output.dir_vec_normval;
      //Calculate absolute error between the L2-measures for the previous and current step
      if(iter > 0){
        abs_norm = std::abs(new_norm - old_norm);
        old_norm = new_norm; 
      }

      std::cout << "\nL2-Norm Step: " << est_output.dir_vec_normval << "\n";
      std::cout << "\nAbsolute L2-Norm between previous and current Step: " << abs_norm << "\n";

      //Collect L2-measures and UAV coordinates
      for(size_t i = 0;i < uavParams.dimension.num_of_uavs;i++){
         ListOfMatrices<T>& uav_truth_list = TruthColl[i];
         Matrix<T> uav_t(3,1);
         ListOfMatrices<T>& uav_guess_list = GuessColl[i];
         Matrix<T> uav_g(3,1);
         for(size_t j = 0;j < 3;j++){
            uav_t(j,0) = UAVTruth(i,j);
            uav_g(j,0) = UAVGuess(i,j);
         }
         uav_truth_list.addElement(uav_t);
         uav_guess_list.addElement(uav_g);
      }
      L2_dirvec_list.addElement(est_output.dir_vec_normval);
      L2_res_list.addElement(est_output.dir_vec_normval);

      auto t_end = std::chrono::steady_clock::now();
      std::chrono::duration<T> elapsed_time = t_end-t_start;
      //Collect elapsed time
      time_list.addElement(elapsed_time.count());
      //The solver converges if the L2-measure for the step is minimal, or that the L2-measures for the step between the previous and current state 
      //is sufficiently small
      if(est_output.dir_vec_normval < uavParams.est.eps || abs_norm < uavParams.est.eps_0){
        MissionStillContinuing = false;
      }

}
std::cout << "\nNumber of iterations: " << iter << "\n";
for(size_t j = 0;j < time_list.getLength();j++){
   time_sum += time_list[j]; 
   }

//Print final results from the UAV Tracking solver
std::cout << "\nInitial Sensor coordinates for UAVs, Ground Truth:\n";
for(size_t i = 0;i < uavParams.dimension.num_of_uavs;i++){
   ListOfMatrices<T>& uav_init_truth_list = TruthColl[i]; 
   std::cout << "\nUAV " << i+1 << ": \n";
   const Matrix<T>& uav_init_t = uav_init_truth_list.getFirstElement();
   uav_init_t.print();
}

std::cout << "\nInitial Sensor coordinates for UAVs, Estimate:\n";
for(size_t i = 0;i < uavParams.dimension.num_of_uavs;i++){
   ListOfMatrices<T>& uav_init_guess_list = GuessColl[i]; 
   std::cout << "\nUAV " << i+1 << ": \n";
   const Matrix<T>& uav_init_g = uav_init_guess_list.getFirstElement();
   uav_init_g.print();
}

std::cout << "\nFinal Sensor coordinates for UAVs, Ground Truth:\n";
for(size_t i = 0;i < uavParams.dimension.num_of_uavs;i++){
   ListOfMatrices<T>& uav_final_truth_list = TruthColl[i]; 
   std::cout << "\nUAV " << i+1 << ": \n";
   const Matrix<T>& uav_final_t = uav_final_truth_list.getLastElement();
   uav_final_t.print();

}

std::cout << "\nFinal Sensor coordinates for UAVs, Estimate:\n";
for(size_t i = 0;i < uavParams.dimension.num_of_uavs;i++){
   ListOfMatrices<T>& uav_final_guess_list = GuessColl[i]; 
   std::cout << "\nUAV " << i+1 << ": \n";
   const Matrix<T>& uav_final_g = uav_final_guess_list.getLastElement();
   uav_final_g.print();
}


for(size_t i = 0;i < uavParams.dimension.num_of_uavs;i++){
   ListOfMatrices<T>& uav_final_guess_list = GuessColl[i];
   ListOfMatrices<T>& uav_final_truth_list = TruthColl[i];
   const Matrix<T>& uav_final_g = uav_final_guess_list.getLastElement();
   const Matrix<T>& uav_final_t = uav_final_truth_list.getLastElement();
   Matrix<T> vec = uav_final_g - uav_final_t;
   T L2norm_UAV = matcalc.L2Norm(vec);
   std::cout<<"\nL2 Norm, UAV " << i+1 << ": " << L2norm_UAV << "\n";

}
//Checks if the converged solution for the estimated UAV positions is stable.
for(size_t i = 0;i < uavParams.dimension.num_of_uavs;i++){
   ListOfMatrices<T>& uav_guess_list = GuessColl[i];
   const Matrix<T>& uav_final_guess = uav_guess_list.getLastElement();
   if(matcalc.L2Norm(uav_final_guess) > uavParams.posParams.radar_radius){
   throw std::runtime_error(
      "The convergent solution is unstable. Reason: Inappropriate choice for step-size h and maximal velocity magnitudes,or poor initial guess."); 
}
}

auto RadarData = std::make_tuple(L2_dirvec_list,L2_res_list,TruthColl,GuessColl,time_list);
//Write results to csv-files
fileWriter(uavParams,RadarData);

std::cout << "\n Total elapsed time for seeking " << uavParams.dimension.num_of_uavs << " UAVs: " << time_sum << " seconds.\n";

}

};