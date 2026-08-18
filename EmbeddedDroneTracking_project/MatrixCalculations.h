//This class performs matrix calculations which mainly focuses on linearly dependence, dot product and decompositions of a matrix
//in order to solve a least square problem, that is QR which uses Givens rotation matrices. 

#include "MatrixObject.h"
#include <tuple>
#include <iostream>
#include <math.h>
#include <cmath>

using namespace std;

template <typename T>

class MatrixCalculations{

public:

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

MatrixObject<T> generateIdentityMatrix(const size_t& n)const {
MatrixObject<T> A(n,n);
for(size_t i = 0;i < A.getRows();i++){
   A(i,i) = 1.0;
}   
return A;
}

auto QRDecomposition_GivensApproach(MatrixObject<T>& A,MatrixObject<T>& b)const{
const T eps = 1e-7;
for(size_t j = 0;j < A.getCols();++j){
   for(size_t i = j + 1;i < A.getRows();++i){
      T val = A(i,j);
      if(abs(val) > eps){
        auto [c,s] = GivensParameters(A(j,j),A(i,j));
        for(size_t k = j;k < 3;k++){
           T R1 = c*A(j,k) + s*A(i,k);
           T R2 = -s*A(j,k) + c*A(i,k);
           A(j,k) = R1;
           A(i,k) = R2;
        }
        T b1_tilde = c*b(j,0) + s*b(i,0);
        T b2_tilde = -s*b(j,0) + c*b(i,0); 
        b(j,0) = b1_tilde;
        b(i,0) = b2_tilde;
      }
   }
}
}

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

MatrixObject<T> LeastSquareSolver_QRDecomposition(MatrixObject<T>& A, MatrixObject<T>& b)const {
const double eps = 1e-7;
QRDecomposition_GivensApproach(A,b);
auto [R_filtered,b_filtered] = generateTruncatedMatrices(A,b);
R_filtered.print();
b_filtered.print();
MatrixObject<T> x = BackwardSubstitution(R_filtered,b_filtered);
return x;
}

};