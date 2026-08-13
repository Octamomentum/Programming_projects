//This class performs matrix calculations which mainly focuses on linearly dependence, dot product and decompositions of a matrix
//in order to solve a least square problem, especially LU/QR where the latter uses Givens rotation matrices. 

#include "MatrixObject.h"
#include <tuple>
#include <iostream>
#include <math.h>
#include <cmath>
#include <cstdlib>

using namespace std;

template <typename T, typename Ti>

class MatrixCalculations{

public:

MatrixObject<T> BackwardSubstitution(const MatrixObject<T>& R, const MatrixObject<T>& b)const {

MatrixObject<T> x(b.getRows(),b.getCols());
for(int i = b.getRows()-1;i >= 0;i--){
   T summation = 0.0;
   for(int j = i;j < R.getCols();j++){
      summation += R(i,j)*x(j,0);
   } 
   x(i,0) = (b(i,0) - summation)/R(i,i);
}    
    
return x;    
}

T square(const T& x)const{
return x*x;
}

T L2Norm(const MatrixObject<T>& res_vec)const{

T res = 0.0;

for(int i = 0;i < res_vec.getRows();i++){
   res += square(res_vec(i,res_vec.getCols()-1)); 
}

return sqrt(res);    
}

auto GivensParameters(const T& r1,const T& r2) const { 
T denum = sqrt(square(r1) + square(r2));
if(denum == 0){
  throw runtime_error("The Givens approach diverges."); 
}
else{
   T c = r1/denum;
   T s = r2/denum;
   return make_tuple(c,s);
}
}

MatrixObject<T> generateIdentityMatrix(const Ti& n)const {
MatrixObject<T> A(n,n);
for(int i = 0;i < A.getRows();i++){
   A(i,i) = 1.0;
}   
return A;
}

auto QRDecomposition_GivensApproach(const MatrixObject<T>& A)const{
const double eps = 1e-7;
MatrixObject<T> R = A;
MatrixObject<T> Q = generateIdentityMatrix(A.getRows());
for(int j = 0;j < A.getCols();++j){
   for(int i = j + 1;i < A.getRows();++i){
      T val = R(i,j);
      if(abs(val) > eps){
        auto [c,s] = GivensParameters(R(j,j),R(i,j));
        for(int k = j;k<A.getCols();++k){
           double r_pivot = R(j,k);
           double r_target = R(i,k);
           R(j,k) = c*r_pivot + s*r_target;
           R(i,k) = c*r_target - s*r_pivot; 
        }
         for(int k = 0;k < A.getRows();++k){
           double q_pivot = Q(k,j);
           double q_target = Q(k,i);
           Q(k,j) = c*q_pivot + s*q_target;
           Q(k,i) = c*q_target - s*q_pivot; 
        }
      }
   }
}
return make_tuple(Q,R);   
}

auto generateTruncatedMatrices(const MatrixObject<T> & A, const MatrixObject<T>& b)const {
int truncated_rows = min(A.getRows(),A.getCols());
MatrixObject<T> A_trunc(truncated_rows,A.getCols());
MatrixObject<T> b_trunc(truncated_rows,b.getCols());

for(int i = 0;i < A_trunc.getRows();i++){
   for(int j = 0;j < A_trunc.getCols();j++){
      A_trunc(i,j) = A(i,j);
   }
   b_trunc(i,b_trunc.getCols()-1) = b(i,b.getCols()-1);
}

return make_tuple(A_trunc, b_trunc);
}

MatrixObject<T> LeastSquareSolver_QRDecomposition(const MatrixObject<T>& A, const MatrixObject<T>& b)const {
const double eps = 1e-7;
auto [Q,R] = QRDecomposition_GivensApproach(A);
MatrixObject<T> b_tilde = Q.Transpose()*b;

auto [R_filtered,b_filtered] = generateTruncatedMatrices(R,b_tilde);

return BackwardSubstitution(R_filtered,b_filtered);
}

};