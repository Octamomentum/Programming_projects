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

MatrixObject<T> distanceMetric(const MatrixObject<T>& L,const MatrixObject<T>& x){
MatrixObject<T> dist_curr(L.getRows(),1);
for(Ti i = 0;i < dist_curr.getRows();i++){
   T summation = 0.0;
   for(Ti j = 0;j < 3;j++){
      summation += matcalc.square(L(i,j) - x(j,0));
   }
   dist_curr(i,0) = sqrt(summation);
}
return dist_curr;   
}

tuple<MatrixObject<T>,MatrixObject<T>> GenerateMatrixData_GN(const MatrixObject<T>& L, const MatrixObject<T> d_meas,const MatrixObject<T>& d_curr, MatrixObject<T>& x){
MatrixObject<T> A(L.getRows(),3);
MatrixObject<T> b(L.getRows(),1);
for(Ti i = 0;i < A.getRows();i++){
   for(Ti j = 0;j < 3;j++){
      A(i,j) = -(L(i,j) - x(j,0))/d_curr(i,0);
   }
}

for(Ti i = 0;i < b.getRows();i++){
   b(i,0) = d_meas(i,0) - d_curr(i,0);
}
return {A,b};
}

tuple<LinkedListObject<T>, LinkedListObject<T>, LinkedListObject<MatrixObject<T>>> TrackingMission_SingleTargetUAV(const MatrixObject<T>& L, const MatrixObject<T>& dist_meas, MatrixObject<T>& x, const Ti& time_limit, const T& eps){
LinkedListObject<T> time_list;
LinkedListObject<T> L2error_list;
LinkedListObject<MatrixObject<T>> x_list;
x_list.addElement(x);
cout << "\nMission started. Tracking down the sensor coordinates for the single-targeted UAV starting...\n";
for(Ti i = 0;i < time_limit;i++){
   cout << "\nTime elapsed: " << i << " seconds...\n";
   time_list.addElement(i);
   MatrixObject<T> d_curr = distanceMetric(L,x);
   auto [A,b] = GenerateMatrixData_GN(L,dist_meas,d_curr,x);
   MatrixObject<T> x_L2 = matcalc.LeastSquareSolver_QRDecomposition(A,b);
   T norm_val = matcalc.L2Norm(x_L2);
   cout << "\nNorm: \n"<< norm_val;
   L2error_list.addElement(norm_val);
   x_list.addElement(x);
   MatrixObject<T> x_new = x_L2.scalarMatrix(0.8);
   x = x + x_new;
   if(abs(norm_val) < eps){
     cout << "\nUAV found!\n";
     cout << "\nSensor coordinates: \n";
     x.print(); 
     break;
   }
   cout << "\nUpdated x\n";
   if(time_limit - i == 1){
     cout << "\nMission failed. Reason: The UAV couldn't be tracked down in time.\n"; 
     cout << "\nThe sensor system found the closest coordinates: \n";
     const MatrixObject<T>& x_est = x_list.getLastElement();
     x_est.print();
   }
}

return {time_list,L2error_list, x_list};   
}

void UAVTracking_Solver(const Ti& num_of_uavs, const MatrixObject<T>& L, const MatrixObject<T>& dist_meas, MatrixObject<T>& x, const Ti& time_limit){
const T eps = 1e-4;
cout << "Welcome to the UAV tracking program. You can use the sensor system to track down any number of UAVs you'd like.\n"; 
cout << "You have chosen a number of " << num_of_uavs <<  " UAVs.\n";
if(num_of_uavs < 2){
   auto [time_list,L2error_list,x_list] = TrackingMission_SingleTargetUAV(L, dist_meas, x, time_limit,eps);
   cout << "\nTime: " << time_list.getLastElement() << " seconds.\n";
   cout << "\nL2 error: " << L2error_list.getLastElement() << "\n";
   cout << "\nSensor coordinates for the UAV: \n";
   const MatrixObject<T>& x_est = x_list.getLastElement();
   x_est.print(); 
}
else{
   throw invalid_argument("Reason: The UAV tracking program isn't developed for tracking down several UAVs yet. Such an update will arrive in the future.");
}  
}

};