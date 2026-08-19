//This class performs matrix calculations which mainly focuses on solving system of equations
//using QR-decomposition which is performed by generating Givens rotation matrices. 

#include "MatrixObject.h"
#include <tuple>
#include <iostream>
#include <math.h>
#include <cmath>

using namespace std;

template <typename T>

class MatrixCalculations{

public:

//Solves a upper-triangular system of equations and returns a solution x

MatrixObject<T> BackwardSubstitution(const MatrixObject<T>& R, const MatrixObject<T>& b) const {
    MatrixObject<T> x(b.getRows(), b.getCols());
    size_t n = b.getRows();

    for (size_t k = 0; k < n; k++) {
        size_t i = n-1-k; 

        T summation = static_cast<T>(0);
        for (size_t j = i + 1; j < R.getCols(); j++) { 
            summation += R(i, j) * x(j, 0);
        } 
        x(i, 0) = (b(i, 0) - summation) / R(i, i);
    }
    return x;    
}

//Square and L2 measure functions

T square(const T& x)const{
return x*x;
}

T L2Norm(const MatrixObject<T>& res_vec)const{

T res = 0.0;

for(size_t i = 0;i < res_vec.getRows();i++){
   res += square(res_vec(i,0)); 
}

return sqrt(res);    
}

//Calculates trigonometric terms cosinus and sinus needed for generate Givens rotations 

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

//QR decomposition of A using Givens approach, which return upper triangular matrix R
//and vector b_tilde that is systematically calculated as Q^Tb, so Q is never needed to
//calculate.

auto QRDecomposition_GivensApproach(MatrixObject<T>& A,MatrixObject<T>& b)const{
MatrixObject<T> R = A;
MatrixObject<T> b_tilde = b;
const T eps = 1e-7;
for(size_t j = 0;j < A.getCols();++j){
   for(size_t i = j + 1;i < A.getRows();++i){
      T val = R(i,j);
      if(abs(val) > eps){
        auto [c,s] = GivensParameters(R(j,j),R(i,j));
        for(size_t k = j;k < 3;k++){
           T R1 = c*R(j,k) + s*R(i,k);
           T R2 = -s*R(j,k) + c*R(i,k);
           R(j,k) = R1;
           R(i,k) = R2;
        }
        T b1_tilde = c*b_tilde(j,0) + s*b_tilde(i,0);
        T b2_tilde = -s*b_tilde(j,0) + c*b_tilde(i,0); 
        b_tilde(j,0) = b1_tilde;
        b_tilde(i,0) = b2_tilde;
      }
   }
}
return make_tuple(R,b_tilde);
}

//This method filtrates the rows of R and b_tilde so that the rows containing zeros of R
//are discarded.
auto generateTruncatedMatrices(const MatrixObject<T> & A, const MatrixObject<T>& b)const {
size_t truncated_rows = min(A.getRows(),A.getCols());
MatrixObject<T> A_trunc(truncated_rows,A.getCols());
MatrixObject<T> b_trunc(truncated_rows,b.getCols());

for(size_t i = 0;i < A_trunc.getRows();i++){
   for(size_t j = 0;j < A_trunc.getCols();j++){
      A_trunc(i,j) = A(i,j);
   }
   b_trunc(i,b_trunc.getCols()-1) = b(i,b.getCols()-1);
}

return make_tuple(A_trunc, b_trunc);
}

//Solver for the least square problems
MatrixObject<T> LeastSquareSolver_QRDecomposition(MatrixObject<T>& A, MatrixObject<T>& b)const {
const double eps = 1e-7;
auto [R,b_tilde] = QRDecomposition_GivensApproach(A,b);
auto [R_filtered,btilde_filtered] = generateTruncatedMatrices(R,b_tilde);
return BackwardSubstitution(R_filtered,btilde_filtered);
}

};