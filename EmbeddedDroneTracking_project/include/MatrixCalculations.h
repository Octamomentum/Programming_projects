/**
 * @file MatrixCalculations.h
 * @brief Performs calculations using matrix structure.
 * 
 * Implements a Gauss-Newton strategy that constructs a Jacobian matrix A and a residual vector b to perform data assimilation later
 * in the UAVTracking file. In the strategy, it calculates residual distances based on the Euclidean measure row-wise between 
 * an overdetermined matrix and a vector to form A and b, which later is used for solving the least square problem using thin QR decomposition of A.
 * The algorithm for the thin QR decomposition method follows the logic of generating Givens rotations. To prevent ill-posed behavior, regularization
 * for the R matrix using a fixed parameter lambda is used in this approach.*/ 

 #pragma once
#include "MatrixObject.h"
#include <tuple>
#include <iostream>
#include <math.h>
#include <cmath>

template <typename T>
using Matrix = MatrixObject<T>;

template <typename T>

class MatrixCalculations{

public:

//Square, dot product and L2 measure functions

T square(const T& x)const{
return x*x;
}

T dot_product(const MatrixObject<T>& vec1,const MatrixObject<T>& vec2)const{

T summation = 0.0;

for(size_t i = 0;i < vec1.getRows();i++){
   summation += vec1(i,0)*vec2(i,0);
}

return summation;
}

T L2Norm(const Matrix<T>& vec)const{
T summation = 0.0;

for(size_t i = 0;i < vec.getRows();i++){
   summation += square(vec(i,0)); 
}
return std::sqrt(summation);    
}

//Method that calculates Euclidean distances relative to the rows of a matrix 
Matrix<T> distanceMetric(const Matrix<T>& mat,const Matrix<T>& vec)const{

Matrix<T> dist(mat.getRows(),1);

for(size_t i = 0;i < mat.getRows();i++){
   T summation = 0.0;
   for(size_t j = 0;j < 3;j++){
      summation += square(vec(j,0) - mat(i,j));
      }
   dist(i,0) = std::sqrt(summation); 
}

return dist;   
}

//Method that constructs a Jacobian matrix A and residual vector b via the Gauss-Newton approach for data assimilation
auto GenerateMatrixData_GN(const Matrix<T>& mat, const Matrix<T> dist_truth,const Matrix<T>& dist_meas,const Matrix<T>& vec_meas)const{
Matrix<T> A(mat.getRows(),3);
Matrix<T> b(mat.getRows(),1);

for(size_t i = 0;i < mat.getRows();i++){
   for(size_t j = 0;j < 3;j++){
      A(i,j) = (vec_meas(j,0)-mat(i,j))/dist_meas(i,0);
   }
}
for(size_t i = 0;i < mat.getRows();i++){
   b(i,0) = dist_truth(i,0) - dist_meas(i,0); 
}
return std::make_tuple(A,b);
}

//Calculates trigonometric terms cosine and sine needed for generating Givens rotations related to QR decomposition.  
auto GivensParameters_QR(const T& pivot_cand,const T& target_cand) const { 
//Used to prevent underflow and overflow
T denum = hypot(pivot_cand,target_cand);
T c = pivot_cand/denum;
T s = target_cand/denum;
return std::make_tuple(c,s);
}



//Implementation for the thin QR-decomposition of any matrix A. In a thin version of the decomposition of A, 
//only the matrix R and the vector b_tilde = Q^Tb are calculated which is achieved by bypassing the calculations of Q explicitly. 

auto QRDecomposition_GivensApproach(const Matrix<T>& A,const Matrix<T>& b, const T& eps)const{
Matrix<T> R = A;
Matrix<T> b_tilde = b;

for(size_t j = 0;j < A.getCols();++j){
for(size_t i = j + 1;i < A.getRows();++i){
T val = R(i,j);
//Pick target candidate that is large enough 
if(std::abs(val) > eps){
   auto [c,s] = GivensParameters_QR(R(j,j),R(i,j));
   //Update R and b_tilde with the multiplied trigonometric terms c and s
   for(size_t k = j;k < A.getCols();k++){
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

return std::make_tuple(R,b_tilde);
}

//Help function that constructs filtered versions of any matrix A and vector b, to form an upper triangular structure. It is here the regularization
//for the matrix R happen after the truncation
auto generateTruncatedMatrices(const Matrix<T> & A, const Matrix<T>& b, const T& eps, const T& lambda)const {
size_t truncated_rows = std::min(A.getRows(),A.getCols());
Matrix<T> A_trunc(truncated_rows,A.getCols());
Matrix<T> b_trunc(truncated_rows,b.getCols());


for(size_t i = 0;i < A_trunc.getRows();i++){
   for(size_t j = 0;j < A_trunc.getCols();j++){
      A_trunc(i,j) = A(i,j);
   }
   b_trunc(i,b_trunc.getCols()-1) = b(i,b.getCols()-1);
}

//Regularization with a fixed parameter lambda
for(size_t i = 0; i < A_trunc.getRows();i++){
   A_trunc(i,i) += lambda;   
}

return std::make_tuple(A_trunc, b_trunc);
}

//Solves an upper-triangular system of equations
Matrix<T> BackwardSubstitution(const Matrix<T>& R, const Matrix<T>& b) const {
    Matrix<T> x(b.getRows(), b.getCols());
    size_t n = b.getRows();

    for (size_t k = 0; k < n; k++) {
        //Bypass when the index is negative 
        size_t i = n-1-k; 

        T summation = static_cast<T>(0);
        for (size_t j = i + 1; j < R.getCols(); j++) { 
            summation += R(i, j) * x(j, 0);
        } 
        x(i, 0) = (b(i, 0) - summation) / R(i, i);
    }
    return x;    
}

//Solver for the entire least square procedure
auto LeastSquareSolver_GaussNewtonQR(const Matrix<T>& mat,const Matrix<T>& x_truth,const Matrix<T>& x_guess, const T& eps, const T& lambda)const {
//Calculate distances relative to the rows of mat
Matrix<T> dist_truth = distanceMetric(mat,x_truth);
Matrix<T> dist_guess = distanceMetric(mat,x_guess);
//Gauss-Newton
auto [A,b] = GenerateMatrixData_GN(mat,dist_truth,dist_guess,x_guess); 
//Perform QR-decomposition of A
auto [R,b_tilde] = QRDecomposition_GivensApproach(A,b,eps);
auto [R_filtered,btilde_filtered] = generateTruncatedMatrices(R,b_tilde,eps,lambda);
//Solve the upper triangular system of equations
Matrix<T> x_sol = BackwardSubstitution(R_filtered,btilde_filtered);
return std::make_tuple(x_sol,btilde_filtered);
}

};