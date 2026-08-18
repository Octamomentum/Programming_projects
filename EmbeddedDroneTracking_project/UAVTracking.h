#include "MatrixCalculations.h"
#include "LinkedListObject.h"
#include "UAVTracking_parameters.h"
#include <iostream>
#include <fstream>
#include <tuple>
#include <random>
#include <math.h>
#include <cmath>
#include <chrono>

using namespace std;

template <typename T>

class UAVTracking{

private:

MatrixCalculations<T> matcalc;

public:

auto generateSeeds(const UAVTracking_parameters& uavParams)const{

MatrixObject<size_t> lm_seed(uavParams.num_of_landmarks,1);
MatrixObject<size_t> uav_ref_seed(uavParams.num_of_uavs,1);
MatrixObject<size_t> uav_guess_seed(uavParams.num_of_uavs,1);

mt19937 gen1(uavParams.landmark_seedvalue);
mt19937 gen2(uavParams.uav_ref_seedvalue);
mt19937 gen3(uavParams.uav_guess_seedvalue);

uniform_int_distribution<size_t> u_g(0,1e7);
for(size_t i = 0; i < uavParams.num_of_landmarks;i++){
   lm_seed(i,0) =  u_g(gen1);
}

for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   uav_ref_seed(i,0) =  u_g(gen2);
}

for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   uav_guess_seed(i,0) =  u_g(gen3);
}


return make_tuple(lm_seed,uav_ref_seed,uav_guess_seed);   
}

auto generateCollections(const UAVTracking_parameters& uavParams)const{
LinkedListObject<LinkedListObject<T>> timeColl;
LinkedListObject<LinkedListObject<T>> L2errorColl;
LinkedListObject<LinkedListObject<MatrixObject<T>>> xColl;
LinkedListObject<LinkedListObject<MatrixObject<T>>> xRefColl;
LinkedListObject<LinkedListObject<T>> dmeasColl;

for(size_t i = 0;i < uavParams.num_of_uavs;i++){
   LinkedListObject<T> time_list;
   LinkedListObject<MatrixObject<T>> x_list;
   LinkedListObject<MatrixObject<T>> x_ref_list;
   LinkedListObject<T> L2error_list;
   LinkedListObject<T> dmeas_list;
   timeColl.addElement(time_list);
   xColl.addElement(x_list);
   xRefColl.addElement(x_ref_list);
   L2errorColl.addElement(L2error_list);
   dmeasColl.addElement(dmeas_list);
}
return make_tuple(L2errorColl,xColl,xRefColl,dmeasColl,timeColl);
}

auto generateRadar(const UAVTracking_parameters& uavParams, const MatrixObject<size_t>& landmark_seed,const MatrixObject<size_t>& uav_ref_seed) const{
if(uavParams.num_of_landmarks < 3){
  throw invalid_argument("Too few landmarks to use the Gauss-Newton solver for the radar system."); 
}
MatrixObject<T> L(uavParams.num_of_landmarks,3);
MatrixObject<T> x(3,uavParams.num_of_uavs);

uniform_real_distribution<T> u_gen(-1,1);
uniform_real_distribution<T> u_radius(0,uavParams.radar_radius);

for(size_t i = 0; i < uavParams.num_of_landmarks;i++){
   mt19937 gen1(landmark_seed(i,0));
   for(size_t j = 0;j < 3;j++){
      L(i,j) = u_gen(gen1);
   }
   T vec_length = sqrt(matcalc.square(L(i,0)) + matcalc.square(L(i,1)) + matcalc.square(L(i,2)));
   T radius = u_radius(gen1);
   for(size_t j = 0;j < 3;j++){
      L(i,j) = radius*(L(i,j)/vec_length);
   }
   
}

for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   mt19937 gen2(uav_ref_seed(i,0));
   for(size_t j = 0;j < 3;j++){
      x(j,i) = u_gen(gen2);
   }
   T vec_length = sqrt(matcalc.square(x(0,i)) + matcalc.square(x(1,i)) + matcalc.square(x(2,i)));
   T radius = u_radius(gen2);
   for(size_t j = 0;j < 3;j++){
      x(j,i) = radius*(x(j,i)/vec_length);
   }
   
}

return make_tuple(L,x);   
}

MatrixObject<T> generateSensorCoordinates(const UAVTracking_parameters& uavParams,const auto& uav_guess_seed) const{

MatrixObject<T> x(3,uavParams.num_of_uavs);

T radar_offset = (uavParams.global_radius-uavParams.radar_radius)/2;

uniform_real_distribution<T> u_gen(-1,1);
uniform_real_distribution<T> u_radius(uavParams.radar_radius+radar_offset,uavParams.global_radius);
for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   mt19937 gen(uav_guess_seed(i,0));
   for(size_t j = 0;j < 3;j++){
      x(j,i) = u_gen(gen);
   }
   T vec_length = sqrt(matcalc.square(x(0,i)) + matcalc.square(x(1,i)) + matcalc.square(x(2,i)));
   T radius = u_radius(gen);
   for(size_t j = 0;j < 3;j++){
      x(j,i) = radius*(x(j,i)/vec_length);
   }
   
}
return x;   
}

MatrixObject<T> distanceMetric(const MatrixObject<T>& L,const MatrixObject<T>& x)const{
MatrixObject<T> dist_curr(L.getRows(),1);

for(size_t i = 0;i < L.getRows();i++){
   T summation = 0.0;
   for(size_t j = 0;j < 3;j++){
      summation += matcalc.square(x(j,0) - L(i,j));
      }
   dist_curr(i,0) = sqrt(summation); 
}

return dist_curr;   
}

auto GenerateMatrixData_GN(const MatrixObject<T>& L, const MatrixObject<T> d_truth,const MatrixObject<T>& d_meas, MatrixObject<T>& x)const{
MatrixObject<T> A(L.getRows(),3);
MatrixObject<T> b(L.getRows(),1);

for(size_t i = 0;i < L.getRows();i++){
   for(size_t j = 0;j < 3;j++){
      A(i,j) = (x(j,0)-L(i,j))/d_meas(i,0);
   }
}
for(size_t i = 0;i < L.getRows();i++){
   b(i,0) = d_truth(i,0) - d_meas(i,0); 
}
return make_tuple(A,b);
}

MatrixObject<T> getUAVVelocity(const UAVTracking_parameters& uavParams, const size_t& heading_ind)const{

MatrixObject<T> vel_vec(3,1);
mt19937 gen(uavParams.initial_velocity_seedvalue+heading_ind);
uniform_real_distribution<T> unit_g(-1,1);
uniform_real_distribution<T> u_g(0,uavParams.max_velocity);
for(size_t i = 0;i < 3;i++){
  vel_vec(i,0) = unit_g(gen);
}
T vel = u_g(gen);
T vel_length = matcalc.L2Norm(vel_vec);
for(size_t i = 0;i < 3;i++){
   vel_vec(i,0) = vel*(vel_vec(i,0)/vel_length);
}
return vel_vec;   
}

auto TrackingMission_SensorCoordinates(const UAVTracking_parameters& uavParams,const auto& L,auto& uav_ref, auto& uav_guess)const{
size_t uav_counter = 0;
auto [L2errorColl,xColl,xRefColl,dmeasColl,timeColl] = generateCollections(uavParams);
LinkedListObject<T> iter_list;
LinkedListObject<T> distanceNorm_list;

for(size_t i = 0;i < uavParams.num_of_uavs;i++){
   LinkedListObject<T>& time_list = timeColl[i];
   time_list.addElement(0.0);
}

for(size_t j = 0;j < uavParams.num_of_uavs;j++){
   MatrixObject<T> x_temp(3,1);
   for(size_t i = 0;i < 3;i++){
      x_temp(i,0) = uav_guess(i,j);  
   }
   LinkedListObject<MatrixObject<T>>& x_list = xColl[j];
   x_list.addElement(x_temp);
}

for(size_t j = 0;j < uavParams.num_of_uavs;j++){
   MatrixObject<T> xref_temp(3,1);
   for(size_t i = 0;i < 3;i++){
      xref_temp(i,0) = uav_ref(i,j);  
   }
   LinkedListObject<MatrixObject<T>>& xref_list = xRefColl[j];
   xref_list.addElement(xref_temp);
}

cout << "\nMission started. Calculating sensor coordinates for the UAVs...\n";
size_t iter = 0;
size_t num_of_headings_counter = 1;
bool stillSearching = true;
MatrixObject<T> vel_vec = getUAVVelocity(uavParams,num_of_headings_counter);
while(stillSearching){
   iter_list.addElement(iter+1);
   cout << "\nIteration " << iter + 1 << "\n";
   for(size_t i = 0; i < uavParams.num_of_uavs;i++){
      auto t_start = chrono::steady_clock::now();
      LinkedListObject<T>& time_list = timeColl[i];
      LinkedListObject<MatrixObject<T>>& x_list = xColl[i];
      LinkedListObject<MatrixObject<T>>& xref_list = xRefColl[i];
      LinkedListObject<T>& L2error_list = L2errorColl[i];
      MatrixObject<T>& uav_g = x_list[iter];
      MatrixObject<T>& uav_r = xref_list[iter];
      if(iter != 0 && iter % uavParams.change_heading == 0){
        num_of_headings_counter++;
        MatrixObject<T> vel_vec = getUAVVelocity(uavParams,num_of_headings_counter);
      }
      if(matcalc.L2Norm(uav_r) > uavParams.radar_radius){
        for(size_t j = 0;j < 3;j++){
           vel_vec(j,0) = -vel_vec(j,0);
        }
      }
      for(size_t j = 0;j < 3;j++){
         uav_r(j,0) += uavParams.h*vel_vec(j,0); 
         uav_g(j,0) += uavParams.h*vel_vec(j,0);
      }      
      MatrixObject<T> d_truth = distanceMetric(L,uav_r);
      MatrixObject<T> d_meas = distanceMetric(L,uav_g);
      auto [A,b] = GenerateMatrixData_GN(L,d_truth,d_meas,uav_g);
      T distmeas_norm_val = matcalc.L2Norm(b);
      LinkedListObject<T>& dmeas_norm_list = dmeasColl[i];
      dmeas_norm_list.addElement(distmeas_norm_val); 
      MatrixObject<T> x_L2 = matcalc.LeastSquareSolver_QRDecomposition(A,b);
      T norm_val = matcalc.L2Norm(x_L2);
      cout << "\nNorm, UAV " << i+1 << ": " << norm_val << "\n";
      L2error_list.addElement(norm_val);
      uav_g = uav_g + x_L2;
      x_list.addElement(uav_g);
      xref_list.addElement(uav_r);
      auto t_end = chrono::steady_clock::now();
      chrono::duration<T> elapsed_time = t_end-t_start;
      T time_val = elapsed_time.count();
      time_list.addElement(time_list.getLastElement() + time_val);

      if(abs(norm_val) < uavParams.eps){
        uav_counter++;    
      }
   }
   
   if(uav_counter == uavParams.num_of_uavs){
     stillSearching = false;
   }
   if(!stillSearching){
      cout << "\nMission completed. All UAVs have been found.\n";
      T time_sum = 0.0;
      for(size_t i = 0;i < uavParams.num_of_uavs;i++){
         const LinkedListObject<T>& time_list = timeColl[i];
         time_sum += time_list.getLastElement();
      } 
      cout << "\nTotal elapsed time: " << time_sum << " seconds.";
      break;
   }
   uav_counter = 0;
   iter += 1;
}

for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   cout << "\nInitial Sensor coordinates for UAV, reference " << i+1 << ": \n";
   LinkedListObject<MatrixObject<T>>& xref_list = xRefColl[i];
   const MatrixObject<T>& x_ref = xref_list.getFirstElement();
   x_ref.print(); 
}
for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   cout << "\nInitial Sensor coordinates for UAV, estimate " << i+1 << ": \n";
   LinkedListObject<MatrixObject<T>>& xest_list = xColl[i];
   const MatrixObject<T>& x_est = xest_list.getFirstElement();
   x_est.print(); 
}
cout << "\nNumber of iterations: " << iter+1 << "\n";
cout << "\nNumber of UAVs found: " << uav_counter << "\n";
for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   cout << "\nSensor coordinates for UAV, reference " << i+1 << ": \n";
   LinkedListObject<MatrixObject<T>>& xref_list = xRefColl[i];
   const MatrixObject<T>& x_ref = xref_list.getLastElement();
   x_ref.print(); 
}

for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   cout << "\nSensor coordinates for UAV, estimated " << i+1 << " : \n";
   LinkedListObject<MatrixObject<T>>& xest_list = xColl[i];
   const MatrixObject<T>& x_est = xest_list.getLastElement();
   x_est.print(); 
}

return make_tuple(iter_list,L2errorColl, xColl,xRefColl,dmeasColl,timeColl);   
}

void fileWriter(const UAVTracking_parameters& uavParams, auto& iter_list, const auto& L2errorCollection, const auto& xCollection, const auto& dmeasCollection, const auto& timeColl,const auto& L) const{

ofstream csv_1 ("L2Convergence_GN.csv");
csv_1 << "Iterations";
for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   csv_1 << ",UAV_" << i << "L2Error";
}
csv_1 << "\n";

for(size_t j = 0;j < iter_list.getLength();j++){
   csv_1 << iter_list[j];
   for(size_t i = 0;i < uavParams.num_of_uavs;i++){
      auto& L2error_list = L2errorCollection[i];
      csv_1 << "," << L2error_list[j];
   }
   csv_1 << "\n";
}

csv_1.close();

ofstream csv_2 ("ResidualErrorConvergence_GN.csv");
csv_2 << "Iterations";
for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   csv_2 << ",UAV_" << i << "ResidualError";
}
csv_2 << "\n";

for(size_t j = 0;j < iter_list.getLength();j++){
   csv_2 << iter_list[j];
   for(size_t i = 0;i < uavParams.num_of_uavs;i++){
      auto& dmeas_list = dmeasCollection[i];
      csv_2 << "," << dmeas_list[j];
   }
   csv_2 << "\n";
}

csv_2.close();


ofstream csv_3 ("Landmarks_GN.csv");
csv_3 << "Landmarkcoord1_" << ",Landmarkcoord2_" <<",Landmarkcoord3_";
csv_3 << "\n";

for(size_t i = 0;i < uavParams.num_of_landmarks;i++){
   csv_3 << L(i,0) << "," << L(i,1) << "," << L(i,2);
   csv_3 << "\n";
}
csv_3.close();


ofstream csv_4 ("UAVConvergence_GN.csv");
for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   csv_4 << "UAVcoord1_" << i << ",UAVcoord2_" << i <<",UAVcoord3_" << i;
}
csv_4 << "\n";

for(size_t i = 0;i < iter_list.getLength();i++){
   for(size_t j = 0;j<uavParams.num_of_uavs;j++){
      auto& x_list = xCollection[j];
      auto x = x_list[i];
      csv_4 << x(0,0) << "," << x(1,0) << "," << x(2,0);
      if(j < uavParams.num_of_uavs-1){
        csv_4 << ","; 
      }
   }
   csv_4 << "\n";
}
csv_4.close();

ofstream csv_5 ("ElapsedTimePerUAV_GN.csv");
csv_5 << "Iterations,";
for(size_t i = 0; i < uavParams.num_of_uavs;i++){
   csv_5 << "UAV_" << i;
   if(i < uavParams.num_of_uavs - i){
     csv_5 << ",";
   }
}
csv_5 << "\n";

for(size_t i = 0;i < iter_list.getLength();i++){
   csv_5 << iter_list[i] << ",";
   for(size_t j = 0;j<uavParams.num_of_uavs;j++){
      auto& time_list = timeColl[j];
      csv_5 << time_list[i];
      if(j < uavParams.num_of_uavs-1){
        csv_5 << ","; 
      }
   }
   csv_5 << "\n";
}
csv_5.close();

}

void UAVTracking_Solver(const UAVTracking_parameters uavParams)const{
if(uavParams.radar_radius > uavParams.global_radius){
  throw invalid_argument("The radar system couldn't be initialized in the simulation. Reason: The radius of the radar is greater than the global environment."); 
}
cout << "\nWelcome to the Single-Targeted UAV Tracking program, STUT. The radar system will search for the sensor coordinates of " << uavParams.num_of_uavs << " UAVs.\n"; 
auto [lm_seed,uav_ref_seed,uav_guess_seed] = generateSeeds(uavParams);
auto [L,uav_ref] = generateRadar(uavParams,lm_seed,uav_ref_seed);
MatrixObject<T> uav_guess = generateSensorCoordinates(uavParams,uav_guess_seed);
auto [time_list,L2errorColl,xColl,xRefColl,dmeasColl,timeColl] = TrackingMission_SensorCoordinates(uavParams,L,uav_ref,uav_guess);
fileWriter(uavParams,time_list,L2errorColl,xColl,dmeasColl,timeColl,L);

}

};