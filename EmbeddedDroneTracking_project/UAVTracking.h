#include "MatrixCalculations.h"
#include "LinkedListObject.h"
#include <iostream>
#include <fstream>
#include <tuple>
#include <random>
#include <functional>
#include <ctime>

using namespace std;

template <typename T, typename Ti>

class UAVTracking{

private:

MatrixCalculations<T> matcalc;

public:

tuple<MatrixObject<T>,MatrixObject<T>> generateRadar(const Ti& num_of_landmarks, const tuple<T,T>& axis_range, const tuple<Ti,Ti>& seeds){

Ti landmark_seed = get<0>(seeds);
Ti ref_seed = get<1>(seeds);
T lb_map = get<0>(axis_range);
T ub_map = get<1>(axis_range);

mt19937 gen1(landmark_seed);
mt19937 gen2(ref_seed);
uniform_real_distribution<T> u_g(lb_map,ub_map);

MatrixObject<T> L(num_of_landmarks,3);
MatrixObject<T> x_ref(3,1);

for(Ti i = 0; i < num_of_landmarks;i++){
   for(Ti j = 0; j < 3;j++){
      L(i,j) = u_g(gen1);
   }
}

for(Ti i = 0;i < 3;i++){
   x_ref(i,0) = u_g(gen2);
}

return {L, x_ref};   
}

MatrixObject<T> distanceMetric(const MatrixObject<T>& L,const MatrixObject<T>& x)const{
MatrixObject<T> dist_curr(L.getRows(),1);
for(Ti i = 0;i < dist_curr.getRows();i++){
   T summation = 0.0;
   for(Ti j = 0;j < 3;j++){
      summation += matcalc.square(x(j,0) - L(i,j));
   }
   dist_curr(i,0) = sqrt(summation);
}
return dist_curr;   
}

tuple<MatrixObject<T>,MatrixObject<T>> GenerateMatrixData_GN(const MatrixObject<T>& L, const MatrixObject<T> d_truth,const MatrixObject<T>& d_meas, MatrixObject<T>& x)const{
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
return {A,b};
}

tuple<LinkedListObject<T>, LinkedListObject<T>, LinkedListObject<MatrixObject<T>>> TrackingMission_SingleTargetUAV(const MatrixObject<T>& L, const MatrixObject<T>& dist_truth,MatrixObject<T>& x, const T& eps)const{
LinkedListObject<T> time_list;
LinkedListObject<T> L2error_list;
LinkedListObject<MatrixObject<T>> x_list;
x_list.addElement(x);
cout << "\nMission started. Tracking down the sensor coordinates for the single-targeted UAV starting...\n";
T delta_time = 0.0;
bool stillSearching = true;
while(stillSearching){
   cout << "\nTime elapsed: " << delta_time << " seconds...\n";
   time_list.addElement(delta_time);
   MatrixObject<T> d_meas = distanceMetric(L,x);
   auto [A,b] = GenerateMatrixData_GN(L,dist_truth,d_meas,x);
   MatrixObject<T> x_L2 = matcalc.LeastSquareSolver_QRDecomposition(A,b);
   cout << "\n Direction vector: \n";
   x_L2.print();
   T norm_val = matcalc.L2Norm(x_L2);
   cout << "\nNorm: \n"<< norm_val;
   L2error_list.addElement(norm_val);
   x = x + x_L2;
   x_list.addElement(x);
   if(abs(norm_val) < eps){
      stillSearching = false;
     cout << "\nUAV found!\n";
     cout << "\nSensor coordinates: \n";
     x.print(); 
     break;
   }
   cout << "\nUpdated x\n";
   x.print();
   if(delta_time == 3600){
     cout << "\nMission failed. Reason: The UAV couldn't be tracked down in time.\n"; 
     cout << "\nThe sensor system found the closest coordinates: \n";
     const MatrixObject<T>& x_est = x_list.getLastElement();
     x_est.print();
   }
   delta_time += 1.0;
}

return {time_list,L2error_list, x_list};   
}

void UAVTracking_Solver(const Ti& num_of_uavs, const MatrixObject<T>& L, const MatrixObject<T>& dist_truth, MatrixObject<T>& uav_guess)const{
const T eps = 1e-4;
cout << "\nWelcome to the UAV tracking program. You can use the sensor system to track down any number of UAVs you'd like.\n"; 
cout << "\nYou have chosen a number of " << num_of_uavs <<  " UAVs.\n";
if(num_of_uavs < 2){
   auto [time_list,L2error_list,x_list] = TrackingMission_SingleTargetUAV(L, dist_truth, uav_guess,eps);
   cout << "\nTime: " << time_list.getLastElement() << " seconds.\n";
   cout << "\nL2 error: " << L2error_list.getLastElement() << "\n";
   cout << "\nSensor coordinates for the UAV: \n";
   const MatrixObject<T>& x_est = x_list.getLastElement();
   x_est.print(); 
   MatrixObject<T> d_xest = distanceMetric(L,x_est);
   cout << "\nEstimated solution, distances\n";
   d_xest.print();
}
else{
   throw invalid_argument("Reason: The UAV tracking program isn't developed for tracking down several UAVs yet. Such an update will arrive in the future.");
}  
}

};