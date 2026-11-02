/**
 * @file UAVTracking.h
 * @brief Radar System Application involving 3D UAV Coordinate Estimation.
 * 
 * Goal: The main objective is to track and estimate coordinates of isolated UAVs in a generated radar environment
 * by using a numerical estimation strategy to minimize the L2-measures of residual distances relative to the landmarks in the monitored radar domain.
 *  The radar system collects valuable information about L2-measured norms for step solution/residuals 
 * and estimated UAV coordinates. Once the strategy has successfully converged, the results are plotted later in Python.
 
 * System setup: 
 * - Initialization parameters: Landmark locations, radar anchor points and initial UAV estimates
 * are all determined in a spherical coordinate system centered at the origin, using fixed pseudo-random seeds.
 * - Boundaries: The landmarks and the physical radar anchor points are generated within a fixed radar radius range while
 * the initial UAV estimates may be generated either outside or inside the radar environment.
 * - Noise sources: In this application, an option to add White Gaussian Noise to the distances of the radar anchor points relative to landmarks is possible. 
 * Kalman filtering: UAVs in this project are assumed to be mutually independent meaning that the covariance matrix P that describes the uncertainty in
 *  the initial state simply is the diagonal matrix with diagonal elements that is the squared symmetric maximal UAV positional offset known
 *  from prior radar measurements.

 * */ 

#pragma once
#include "MatrixCalculations.h"
#include "StateEstimator.h"
#include "generateRadarStructure.h"
#include "LinkedListObject.h"
#include "TrackingParameters.h"
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

public:

MatrixCalculations<T> matcalc;
StateEstimator<T> state_est;
generateRadarStructure<T> radar;

//Generate collection of empty lists for the UAV tracking application
auto generateCollections(const size_t& num_of_uavs)const{

CollOfMatrices<T> UAVColl;

for(size_t i = 0;i < num_of_uavs;i++){
   ListOfMatrices<T> UAVList;

   UAVColl.addElement(UAVList);
}

return UAVColl;

}


//Writes numerical results to csv-files which will be used later for plots in Python.
void fileWriter(const UAVTracking_parameters& uavParams,const std::tuple<List<T>,List<T>, CollOfMatrices<T>,List<T>>& RadarData,
    const Matrix<T>& RadarAnchors) const{
List<T> L2Norm_SolList = std::get<0>(RadarData);
List<T> L2Norm_ResList = std::get<1>(RadarData);
CollOfMatrices<T> UAVColl = std::get<2>(RadarData);
List<T> time_list = std::get<3>(RadarData);
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

//Write L2-measures of the step solution over iterations to csv-file
std::ofstream csv_1 ("../output_data/L2Convergence_" + method + "," + noiseAnswer + ".csv");
csv_1 << "Iterations," << "L2Error\n";
for(size_t j = 0;j < num_of_iterations;j++){
   csv_1 << j+1 << "," << L2Norm_SolList[j] << "\n";  
}
csv_1.close();

//Write L2-measures of the residual distances over iterations to csv-file
std::ofstream csv_2 ("../output_data/ResidualErrorConvergence_" + method + "," + noiseAnswer + ".csv");
csv_2 << "Iterations," << "ResidualError\n";
for(size_t j = 0;j < num_of_iterations;j++){
   csv_2 << j+1 << "," << L2Norm_ResList[j] << "\n";
  
}
csv_2.close();

//Write the UAV estimates over iterations per UAV to csv-file
std::ofstream csv_3 ("../output_data/UAVEstimates_" + method + "," + noiseAnswer + ".csv");
csv_3 << "Iterations, ";
for(size_t i = 0;i < uavParams.dimension.num_of_uavs;i++){
   csv_3 << "EstCoord1_UAV" << i << ",EstCoord2_UAV" << i <<",EstCoord3_UAV" << i;
   if(i < uavParams.dimension.num_of_uavs-1){
     csv_3 << ","; 
   }
}
csv_3 << "\n";
for(size_t i = 0;i < num_of_iterations;i++){
   csv_3 << i+1 << ",";
for(size_t j = 0;j < uavParams.dimension.num_of_uavs;j++){
   auto& UAVList = UAVColl[j];
   auto& uav = UAVList[i];
   csv_3 << uav(0,0) << "," << uav(1,0) << "," << uav(2,0);
   if(j < uavParams.dimension.num_of_uavs-1){
      csv_3 << ",";
   }      
}
csv_3 << "\n";
}     
csv_3.close();

//Write the physical radar anchor points to csv-file
std::ofstream csv_4 ("../output_data/RadarAnchors.csv");
for(size_t i = 0;i < RadarAnchors.getRows();i++){
   csv_4 << "Coord1_RadarAnchor" << i << ",Coord2_RadarAnchor" << i << "Coord3_RadarAnchor" << i;
   if(i < uavParams.dimension.num_of_uavs-1){
     csv_4 << ","; 
   }

}
csv_4 << "\n";
for(size_t i = 0;i < RadarAnchors.getRows();i++){
   Matrix<T> radar_anchor = RadarAnchors.getRow(i);
   csv_4 << radar_anchor(0,0) << "," << radar_anchor(1,0) << "," << radar_anchor(2,0);
   if(i < uavParams.dimension.num_of_uavs-1){
     csv_4 << ","; 
   }
}
csv_4 << "\n";
csv_4.close();



//Write elapsed time for using a numerical estimation method solving the UAV Tracking problem over iterations to csv-file
std::ofstream csv_5 ("../output_data/ElapsedTime_" + method + "," + noiseAnswer + ".csv");
csv_5 << "Iterations, Time\n";
for(size_t j = 0;j < num_of_iterations;j++){
   csv_5 << j+1 << "," << time_list[j] << "\n";
}
csv_5.close();
}


//The radar system logic behind the UAV Tracking problem
void TrackingMission_SensorCoordinates(const UAVTracking_parameters& uavParams,Matrix<T>& LandmarkMatrix, Matrix<T>& UAV,const Matrix<T>& RadarAnchors)
const{

std::cout << "\nMission started. Collecting radar data related to the locations of the UAVs...\n";

T time_sum = 0.0;

List<T> time_list;
List<T> L2_dirvec_list;
List<T> L2_res_list;
auto UAVColl = generateCollections(uavParams.dimension.num_of_uavs);

//Initialize the UAV estimates for the first state

for(size_t i = 0; i < uavParams.dimension.num_of_uavs;i++){
   ListOfMatrices<T>& UAVList = UAVColl[i];
   Matrix<T> uav = UAV.getRow(i); 
   UAVList.addElement(uav);
}

T lambda = 0.0;
size_t iter = 0;

//Initialize covariance matrix P for the Kalman filtering strategy
Matrix<T> IdentityMatrix_termP = matcalc.getIdentityMatrix(3*uavParams.dimension.num_of_uavs);
//P is treated initially as the diagonal matrix with diagonal elements as the squared maximal UAV positional error measured in meters, 
//meaning P measures uncertainty in the initial state for the UAVs
Matrix<T> P = IdentityMatrix_termP.scalarMatrix(matcalc.square(uavParams.posParams.maximal_offset));

TrackingParameters TrackingParams{LandmarkMatrix,UAV,P,iter,lambda};
//Generate radar measurements for the UAV tracking application
Matrix<T> distRadar = matcalc.generateRadarMeasurements(uavParams,TrackingParams.LandmarkMatrix,RadarAnchors);

bool MissionStillContinuing = true;
 //Solve the UAV Tracking problem in order to collect L2-norms of the step solution and residual distances, elapsed time using a chosen method,
//and the corresponding UAV estimates
while(MissionStillContinuing){
      std::cout << "\nIteration " << TrackingParams.iter+1 << "\n";
      auto t_start = std::chrono::steady_clock::now(); 
      
      //Solve the numerical estimation problem per iteration
      auto [dir_vec_normval,res_normval] = state_est.State_Estimator(uavParams,TrackingParams,distRadar,iter);

      std::cout << "\nL2-Norm Step: " << dir_vec_normval << "\n";
      std::cout << "\nL2-Norm Residuals: " << res_normval << "\n";

      //Collect L2-measures and UAV coordinates
      for(size_t i = 0;i < uavParams.dimension.num_of_uavs;i++){
         ListOfMatrices<T>& uav_list = UAVColl[i];
         Matrix<T> uav = TrackingParams.UAV.getRow(i);
         uav_list.addElement(uav);
      }
      L2_dirvec_list.addElement(dir_vec_normval);
      L2_res_list.addElement(dir_vec_normval);

      auto t_end = std::chrono::steady_clock::now();
      std::chrono::duration<T> elapsed_time = t_end-t_start;
      //Collect elapsed time
      time_list.addElement(elapsed_time.count());

      //The solver converges if the L2-measure for the step solution is minimal
      if(dir_vec_normval < uavParams.est.eps){
        MissionStillContinuing = false;
      }
      TrackingParams.iter++;

}
std::cout << "\nNumber of iterations: " << TrackingParams.iter << "\n";
for(size_t j = 0;j < time_list.getLength();j++){
   time_sum += time_list[j]; 
   }

//Print final results from the UAV Tracking solver

std::cout << "\nInitial UAV coordinates, Estimate:\n";
for(size_t i = 0;i < uavParams.dimension.num_of_uavs;i++){
   ListOfMatrices<T>& uav_init_list = UAVColl[i]; 
   std::cout << "\nUAV " << i+1 << ": \n";
   const Matrix<T>& uav_init = uav_init_list.getFirstElement();
   uav_init.print();
}

std::cout << "\nRadar Anchors:\n";
for(size_t i = 0;i < uavParams.dimension.num_of_uavs;i++){
   Matrix<T> radar_anchor = RadarAnchors.getRow(i);
   std::cout << "\nGround station " << i+1 << ": \n";
   radar_anchor.print();
}

std::cout << "\nFinal UAV coordinates, Estimate:\n";
for(size_t i = 0;i < uavParams.dimension.num_of_uavs;i++){
   ListOfMatrices<T>& uav_final_guess_list = UAVColl[i]; 
   std::cout << "\nUAV " << i+1 << ": \n";
   const Matrix<T>& uav_final_g = uav_final_guess_list.getLastElement();
   uav_final_g.print();
}

//Computing the numerical accuracy for the converged UAV estimates
for(size_t i = 0;i < uavParams.dimension.num_of_uavs;i++){
   ListOfMatrices<T>& uav_list = UAVColl[i];
   const Matrix<T>& uav_final = uav_list.getLastElement();
   Matrix<T> radar_anchor = RadarAnchors.getRow(i);
   T L2norm = matcalc.L2Norm(uav_final - radar_anchor);
   std::cout << "\n L2-norm, UAV " << i+1 << ":" << L2norm << "\n";
}

auto RadarData = std::make_tuple(L2_dirvec_list,L2_res_list,UAVColl,time_list);
//Write results to csv-files
fileWriter(uavParams,RadarData,RadarAnchors);

std::cout << "\n Total elapsed time for seeking " << uavParams.dimension.num_of_uavs << " UAVs: " << time_sum << " seconds.\n";

}

};