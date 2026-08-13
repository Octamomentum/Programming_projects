#include "MatrixCalculations.h"
#include "LinkedListObject.h"
#include "UAVTracking_parameters.h"
#include <iostream>
#include <fstream>
#include <tuple>
#include <random>
#include <functional>
#include <ctime>
#include <math.h>
#include <cmath>

using namespace std;

template <typename T, typename Ti>

class UAVTracking{

private:

MatrixCalculations<T,Ti> matcalc;
UAVTracking_parameters uav_params;

public:

auto generateSeeds()const{

MatrixObject<Ti> lm_seed(uav_params.num_of_landmarks,1);
MatrixObject<Ti> uav_ref_seed(uav_params.num_of_uavs,1);
MatrixObject<Ti> uav_guess_seed(uav_params.num_of_uavs,1);

random_device rd;
mt19937 gen(rd());
uniform_int_distribution<Ti> u_g(0,1e7);
for(Ti i = 0; i < uav_params.num_of_landmarks;i++){
   lm_seed(i,0) =  u_g(gen);
}

for(Ti i = 0; i < uav_params.num_of_uavs;i++){
   uav_ref_seed(i,0) =  u_g(gen);
}

for(Ti i = 0; i < uav_params.num_of_uavs;i++){
   uav_guess_seed(i,0) =  u_g(gen);
}

return make_tuple(lm_seed,uav_ref_seed,uav_guess_seed);   
}

auto generateCollections()const{
LinkedListObject<LinkedListObject<T>> L2errorColl;
LinkedListObject<LinkedListObject<MatrixObject<T>>> xColl;
LinkedListObject<LinkedListObject<T>> dmeasColl;

for(Ti i = 0;i < uav_params.num_of_uavs;i++){
   LinkedListObject<MatrixObject<T>> x_list;
   LinkedListObject<T> L2error_list;
   LinkedListObject<T> dmeas_list;
   xColl.addElement(x_list);
   L2errorColl.addElement(L2error_list);
   dmeasColl.addElement(dmeas_list);
}
return make_tuple(L2errorColl,xColl,dmeasColl);
}

auto generateRadar(MatrixObject<Ti>& landmark_seed, MatrixObject<Ti>& uav_ref_seed) const{

MatrixObject<T> L(uav_params.num_of_landmarks,3);
MatrixObject<T> x(3,uav_params.num_of_uavs);


uniform_real_distribution<T> unit_gen(-1,1);

for(Ti i = 0; i < uav_params.num_of_landmarks;i++){
   mt19937 gen1(landmark_seed(i,0));
   for(Ti j = 0;j < 3;j++){
      L(i,j) = unit_gen(gen1);
   }
   T vec_length = sqrt(matcalc.square(L(i,0)) + matcalc.square(L(i,1)) + matcalc.square(L(i,2)));
   for(Ti j = 0;j < 3;j++){
      L(i,j) = uav_params.radar_radius*(L(i,j)/vec_length);
   }
}

for(Ti i = 0; i < uav_params.num_of_uavs;i++){
   mt19937 gen2(uav_ref_seed(i,0));
   for(Ti j = 0;j < 3;j++){
      x(j,i) = unit_gen(gen2);
   }
   T vec_length = sqrt(matcalc.square(x(0,i)) + matcalc.square(x(1,i)) + matcalc.square(x(2,i)));
   for(Ti j = 0;j < 3;j++){
      x(j,i) = uav_params.radar_radius*(x(j,i)/vec_length);
   }
}

return make_tuple(L,x);   
}

MatrixObject<T> generateSensorCoordinates(MatrixObject<Ti>& uav_guess_seed) const{

MatrixObject<T> x(3,uav_params.num_of_uavs);

uniform_real_distribution<T> unit_gen(-1,1);

for(Ti i = 0; i < uav_params.num_of_uavs;i++){
   mt19937 gen2(uav_guess_seed(i,0));
   for(Ti j = 0;j < 3;j++){
      x(j,i) = unit_gen(gen2);
   }
   T vec_length = sqrt(matcalc.square(x(0,i)) + matcalc.square(x(1,i)) + matcalc.square(x(2,i)));
   for(Ti j = 0;j < 3;j++){
      x(j,i) = (uav_params.radar_radius+uav_params.radar_offset)*(x(j,i)/vec_length);
   }
}

return x;   
}

MatrixObject<T> distanceMetric(const MatrixObject<T>& L,const MatrixObject<T>& x)const{
MatrixObject<T> dist_curr(L.getRows(),1);

for(Ti i = 0;i < L.getRows();i++){
   T summation = 0.0;
   for(Ti j = 0;j < 3;j++){
      summation += matcalc.square(x(j,0) - L(i,j));
      }
   dist_curr(i,0) = sqrt(summation); 
}

return dist_curr;   
}

auto GenerateMatrixData_GN(const MatrixObject<T>& L, const MatrixObject<T> d_truth,const MatrixObject<T>& d_meas, MatrixObject<T>& x)const{
MatrixObject<T> A(L.getRows(),3);
MatrixObject<T> b(L.getRows(),1);

for(Ti i = 0;i < L.getRows();i++){
   for(Ti j = 0;j < 3;j++){
      A(i,j) = (x(j,0)-L(i,j))/d_meas(i,0);
   }
}
for(Ti i = 0;i < L.getRows();i++){
   b(i,0) = d_truth(i,0) - d_meas(i,0); 
}
return make_tuple(A,b);
}

auto TrackingMission_SensorCoordinates(const auto& L,const auto& uav_ref, auto& uav_guess)const{
Ti uav_counter = 0;
auto [L2errorColl,xColl,dmeasColl] = generateCollections();
LinkedListObject<T> time_list;
LinkedListObject<T> distanceNorm_list;

for(Ti j = 0;j < uav_params.num_of_uavs;j++){
   MatrixObject<T> x_temp(3,1);
   for(Ti i = 0;i < 3;i++){
      x_temp(i,0) = uav_guess(i,j);  
   }
   LinkedListObject<MatrixObject<T>>& x_list = xColl[j];
   x_list.addElement(x_temp);
}

cout << "\nMission started. Tracking down the sensor coordinates for the UAVs...\n";
Ti delta_time = 0;
bool stillSearching = true;
while(stillSearching){
   time_list.addElement(delta_time);
   for(Ti i = 0; i < uav_params.num_of_uavs;i++){
      cout << "\nTime elapsed: " << delta_time << "\n";
      LinkedListObject<MatrixObject<T>>& x_list = xColl[i];
      LinkedListObject<T>& L2error_list = L2errorColl[i];
      MatrixObject<T>& uav = x_list[delta_time];
      MatrixObject<T> uav_refvec(3,1);
      for(Ti j = 0;j < 3;j++){
         uav_refvec(j,0) = uav_ref(j,i);
      }
      MatrixObject<T> d_truth = distanceMetric(L,uav_refvec);
      MatrixObject<T> d_meas = distanceMetric(L,uav);
      auto [A,b] = GenerateMatrixData_GN(L,d_truth,d_meas,uav);
      T distmeas_norm_val = matcalc.L2Norm(b);
      LinkedListObject<T>& dmeas_norm_list = dmeasColl[i];
      dmeas_norm_list.addElement(distmeas_norm_val); 
      MatrixObject<T> x_L2 = matcalc.LeastSquareSolver_QRDecomposition(A,b);
      T norm_val = matcalc.L2Norm(x_L2);
      cout << "\nNorm: " << norm_val << "\n";
      L2error_list.addElement(norm_val);
      uav = uav + x_L2;
      x_list.addElement(uav);
      if(abs(norm_val) < uav_params.eps){
        uav_counter++;    
      }
   }
   
   delta_time += 1.0;
   if(uav_counter == uav_params.num_of_uavs){
     stillSearching = false;
   }
   if(!stillSearching){
      cout << "\nMission completed. All UAVs have been found.\n";
      break;
   }
   uav_counter = 0;
}

for(Ti i = 0;i < uav_params.num_of_uavs;i++){
   LinkedListObject<T>& norm_list = dmeasColl[i];
   norm_list.print();
}

cout << "\nNumber of UAVs found: " << uav_counter << "\n";
cout << "\nTime: " << time_list.getLastElement() << " seconds.\n";
cout << "\nSensor coordinates for UAV, reference\n";
for(Ti j = 0; j < uav_params.num_of_uavs;j++){
   cout << "\nUAV " << j+1 << ": \n";
   for(Ti i = 0;i < 3;i++){
      cout << "\n(" << j << "," << i << "): " << uav_ref(i,j) << "\n";
   }
}

for(Ti i = 0; i < uav_params.num_of_uavs;i++){
   cout << "\nSensor coordinates for UAV, estimated " << i+1 << " : \n";
   LinkedListObject<MatrixObject<T>>& x_list = xColl[i];
   const MatrixObject<T>& x_est = x_list.getLastElement();
   x_est.print(); 
}

return make_tuple(time_list,L2errorColl, xColl,dmeasColl);   
}

void UAVTracking_Solver()const{
cout << "\nWelcome to the UAV tracking program. You can use the sensor system to track down any number of UAVs you'd like.\n"; 
cout << "\nYou have chosen a number of " << uav_params.num_of_uavs <<  " UAVs.\n";
auto [lm_seed,uav_ref_seed,uav_guess_seed] = generateSeeds();
auto [L,uav_ref] = generateRadar(lm_seed,uav_ref_seed);
MatrixObject<T> uav_guess = generateSensorCoordinates(uav_guess_seed);
auto [time_list,L2errorColl,xColl,dmeasColl] = TrackingMission_SensorCoordinates(L,uav_ref,uav_guess);

ofstream csv_1 ("L2Convergence_GN.csv");
csv_1 << "Time";
for(Ti i = 0; i < uav_params.num_of_uavs;i++){
   csv_1 << ",UAV_" << i << "L2Error";
}
csv_1 << "\n";

for(Ti j = 0;j < time_list.getLength();j++){
   csv_1 << time_list[j];
   for(Ti i = 0;i < uav_params.num_of_uavs;i++){
      auto& L2error_list = L2errorColl[i];
      csv_1 << "," << L2error_list[j];
   }
   csv_1 << "\n";
}

csv_1.close();

ofstream csv_2 ("ResidualErrorConvergence_GN.csv");
csv_2 << "Time";
for(Ti i = 0; i < uav_params.num_of_uavs;i++){
   csv_2 << ",UAV_" << i << "ResidualError";
}
csv_2 << "\n";

for(Ti j = 0;j < time_list.getLength();j++){
   csv_2 << time_list[j];
   for(Ti i = 0;i < uav_params.num_of_uavs;i++){
      auto& dmeas_list = dmeasColl[i];
      csv_2 << "," << dmeas_list[j];
   }
   csv_2 << "\n";
}

csv_2.close();

ofstream csv_3 ("UAVReferencepoints.csv");
for(Ti i = 0; i < uav_params.num_of_uavs;i++){
   csv_3 << "UAVrefcoord1_" << i << ",UAVrefcoord2_" << i <<",UAVrefcoord3_" << i;
}
csv_3 << "\n";

for(Ti i = 0;i < uav_params.num_of_uavs;i++){
   csv_3 << uav_ref(0,i) << "," << uav_ref(1,i) << "," << uav_ref(2,i);
   if(i < uav_params.num_of_uavs - 1){
      csv_3 << ",";
   }
}
csv_3 << "\n";

csv_3.close();


ofstream csv_4 ("Landmarks_GN.csv");
csv_4 << "Landmarkcoord1_" << ",Landmarkcoord2_" <<",Landmarkcoord3_";
csv_4 << "\n";

for(Ti i = 0;i < uav_params.num_of_landmarks;i++){
   csv_4 << L(i,0) << "," << L(i,1) << "," << L(i,2);
   csv_4 << "\n";
}
csv_4.close();


ofstream csv_5 ("UAVConvergence_GN.csv");
for(Ti i = 0; i < uav_params.num_of_uavs;i++){
   csv_5 << "UAVcoord1_" << i << ",UAVcoord2_" << i <<",UAVcoord3_" << i;
}
csv_5 << "\n";

for(Ti i = 0;i < time_list.getLength();i++){
   for(Ti j = 0;j<uav_params.num_of_uavs;j++){
      auto& x_list = xColl[j];
      auto x = x_list[i];
      csv_5 << x(0,0) << "," << x(1,0) << "," << x(2,0);
      if(j < uav_params.num_of_uavs-1){
        csv_5 << ","; 
      }
   }
   csv_5 << "\n";
}
csv_5.close();

}

};