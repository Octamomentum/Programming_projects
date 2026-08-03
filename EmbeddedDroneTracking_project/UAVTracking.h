#include "MatrixCalculations.h"
#include "LinkedListObject.h"
#include <iostream>
#include <fstream>
#include <tuple>
#include <random>

using namespace std;

template <typename T, int num_of_uavs, int num_of_landmarks>

class UAVTracking{

private:

LinkedListObject<T> time_axis_lst;
MatrixCalculations<T> matcalc;

public:

MatrixObject<T> generateLandmarks(const int& landmark_seed, const tuple<T,T>& axis_range)const{
if(num_of_landmarks < 3){
  throw invalid_argument("Too few landmarks to solve the problem with QR decomposition.");  
}
else{
   MatrixObject<T> lm_matrix(num_of_landmarks,3);
   T lb_map = get<0>(axis_range);
   T ub_map = get<1>(axis_range);
   mt19937 gen(landmark_seed);
   uniform_real_distribution<T> u_g(lb_map,ub_map);
   for(int i = 0;i < lm_matrix.getRows();i++){
       for(int j = 0;j < lm_matrix.getCols();j++){
          T val = u_g(gen); 
          lm_matrix(i,j) = val; 
       }
   }
   return lm_matrix;
   }
}

MatrixObject<T> generate_x(const int& uav_seed, const tuple<T,T>& axis_range)const{
MatrixObject<T> x(3,1);
T lb_map = get<0>(axis_range);
T ub_map = get<1>(axis_range);
mt19937 gen(uav_seed);
uniform_real_distribution<T> u_g(lb_map,ub_map);
for(int i = 0;i < x.getRows();i++){
   T val = u_g(gen); 
   x(i,0) = val; 
}
return x;
}

tuple<MatrixObject<T>,MatrixObject<T>> generateMatrixData(const MatrixObject<T>& L, const MatrixObject<T>& x, const tuple<T,T>& axis_range)const{
MatrixObject<T> A = L;
MatrixObject<T> b(num_of_landmarks,1);
T lb_map = get<0>(axis_range);
T ub_map = get<1>(axis_range);
for(int i = 0; i < num_of_landmarks;i++){
   T summation = 0.0; 
   for(int j = 0;j < 3;j++){
      summation += matcalc.square(A(i,j) - x(j,0));
      }
      b(i,0) = sqrt(summation);
   }

return {A,b};
}

void OperationUAVTracking(MatrixObject<T>& A, MatrixObject<T>& b, MatrixObject<T>& x){
for(int i = 0; i < num_of_uavs;i++){
   LinkedListObject<double> L2_error_lst; 
   MatrixObject<T> x_true = matcalc.LeastSquareSolver_QRDecomposition(A,b);
   double norm_value = matcalc.L2Norm(A*x_true - b);
   L2_error_lst.addElement(norm_value);
   cout << "L2 norm for UAV " << i+1 << ": " << norm_value;
}
}

};