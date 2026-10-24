/**
 * @file MatrixCalculations.h
 * @brief Performs calculations using matrix structure.
 * 
 * Implements methods using matrix calculations, including the construction of Jacobian matrix J and a residual vector r to perform data assimilation,
 *  and also QR-decomposition of overdetermined matrices based on the generation of Givens rotations.
  * 
 * */ 

 #pragma once
#include "UAVTracking_parameters.h"
#include "TrackingParameters.h"
#include "GenerateWGNSource.h"

#include <tuple>
#include <iostream>
#include <math.h>
#include <cmath>
#include <iostream>

template <typename T>
using Matrix = MatrixObject<T>;

template <typename T>
class MatrixCalculations{

private:

GenerateWGNSource<T> generate_wgn;

public:

//Help method for initializing lambda that is used in the StateEstimator class, which does retrieve the largest element from a vector

T maximal_vector_element(const Matrix<T>& vec)const{
T max_val = 0.0;
for(size_t i = 0;i < vec.getRows();i++){
   max_val = (max_val > vec(i,0)) ? max_val:vec(i,0); 
}
return max_val;  
}

//Squaring a number

T square(const T& x)const{
return x*x;
}

//Dot product

T dot_product(const MatrixObject<T>& vec1,const MatrixObject<T>& vec2)const{
T summation = 0.0;
for(size_t i = 0;i < vec1.getRows();i++){
   summation += vec1(i,0)*vec2(i,0);
}
return summation;
}

//Euclidean Norm

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

//Method that transforms a vectorized matrix object to an actual matrix structure

Matrix<T> transformVectorToMatrix(const Matrix<T>& vec,const size_t& n)const{

size_t m = vec.getRows()/n;

Matrix<T> mat(n,m);
size_t counter = 0;

for(size_t i = 0;i < n;i++){
   for(size_t j = 0;j < m;j++){
      mat(i,j) = vec(counter,0);
      counter++;
   }
}
return mat;
}

//Method that constructs a block matrix of Jacobians and a vectorized residual vector for data assimilation relating to estimating UAV positions
auto GenerateMatrixData(const UAVTracking_parameters& uavParams, const TrackingParameters& trackingParams, const Matrix<T>& guess,
    const std::tuple<Matrix<size_t>,size_t> varianceParams)const{

Matrix<size_t> VarianceSeed = std::get<0>(varianceParams);
size_t iter = std::get<1>(varianceParams);
//Initialize a block matrix whose diagonal elements will consist of Jacobian relating to each UAV
size_t m = uavParams.dimension.num_of_landmarks*uavParams.dimension.num_of_uavs;
size_t n = 3*uavParams.dimension.num_of_uavs;
Matrix<T> J(m,n);
Matrix<T> r(m,1);
//For each UAV, calculate Jacobian and residuals
for(size_t uav_ind = 0;uav_ind < uavParams.dimension.num_of_uavs;uav_ind++){
  Matrix<T> uav_truth(3,1);
  Matrix<T> uav_guess(3,1);
  for(size_t i = 0; i < 3;i++){
    uav_truth(i,0) = trackingParams.UAVTruth(uav_ind,i);
    uav_guess(i,0) = guess(uav_ind,i);  
  }
  Matrix<T> dist_truth = distanceMetric(trackingParams.mat,uav_truth);
  //Generate WGN if option is available
  if(uavParams.options.applyWGN){
    size_t var_ind = VarianceSeed(uav_ind,0) + iter;
    generate_wgn.GenerateWGN(dist_truth,var_ind,uavParams.variance.variance_distances);
  }
  Matrix<T> dist_guess = distanceMetric(trackingParams.mat,uav_guess); 
  for(size_t i = 0;i < uavParams.dimension.num_of_landmarks;i++){
     for(size_t j = 0;j < 3;j++){
        J(i + uavParams.dimension.num_of_landmarks*uav_ind,3*uav_ind+j) = (uav_guess(j,0)-trackingParams.mat(i,j))/dist_guess(i,0);
     }
   }

  for(size_t i = 0;i < uavParams.dimension.num_of_landmarks;i++){
     r(i + uavParams.dimension.num_of_landmarks*uav_ind,0) = dist_truth(i,0) - dist_guess(i,0); 
   }
}
return std::make_tuple(J,r);
}

//Augment J and r if Levenberg-Marquardt method is applied to the UAV Tracking problem solver, otherwise keep the Jacobian and residual
auto generateMatrixForm(const Matrix<T>& A,const Matrix<T>& b, const T& lambda, const std::string& activate_method)const{

if(activate_method == "L-M"){
  Matrix<T> newJacobian(A.getRows()+A.getCols(),A.getCols());
  Matrix<T> newResidual(A.getRows()+A.getCols(),1);
  for(size_t i = 0;i < A.getRows();i++){
      for(size_t j = 0;j < A.getCols();j++){
          newJacobian(i,j) = A(i,j);
      }
      newResidual(i,0) = b(i,0);
   }
   for(size_t i = 0; i < newJacobian.getCols();i++){
      newJacobian(i+A.getRows(),i) = std::sqrt(lambda); 
    }
   return std::make_tuple(newJacobian,newResidual);  
}
return std::make_tuple(A,b);
}

//Calculates trigonometric terms cosine and sine needed for generating Givens rotations related to QR decomposition.  
auto GivensParameters_QR(const T& pivot_cand,const T& target_cand) const { 
//Used to prevent underflow and overflow
T denum = std::hypot(pivot_cand,target_cand);
T c = pivot_cand/denum;
T s = target_cand/denum;
return std::make_tuple(c,s);
}

//Implementation for the QR-decomposition of any matrix A. In this implementation of the QR-decomposition, 
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
   for(size_t k = 0;k < b_tilde.getCols();k++){

      T b1_tilde = c*b_tilde(j,k) + s*b_tilde(i,k);
      T b2_tilde = -s*b_tilde(j,k) + c*b_tilde(i,k); 
      b_tilde(j,k) = b1_tilde;
      b_tilde(i,k) = b2_tilde;
      }
   
   }
   }
}
return std::make_tuple(R,b_tilde);
}

//Construct an identity matrix
Matrix<T> getIdentityMatrix(const size_t& m)const{

Matrix<T> IdentityMatrix(m,m);

for(size_t i = 0; i < m;i++){
   IdentityMatrix(i,i) = 1.0;
}

return IdentityMatrix;
}

//Help function that constructs filtered versions of any matrix A and vector b, to form an upper triangular structure.

auto generateTruncatedMatrices(const Matrix<T> & A, const Matrix<T>& b)const {
size_t truncated_rows = std::min(A.getRows(),A.getCols());
Matrix<T> A_trunc(truncated_rows,A.getCols());
Matrix<T> b_trunc(truncated_rows,1);


for(size_t i = 0;i < A_trunc.getRows();i++){
   for(size_t j = 0;j < A_trunc.getCols();j++){
      A_trunc(i,j) = A(i,j);
   }
   b_trunc(i,0) = b(i,0);
}

return std::make_tuple(A_trunc, b_trunc);
}

//Solves an upper-triangular system of equations
Matrix<T> BackwardSubstitution(const Matrix<T>& R, const Matrix<T>& b) const {
    Matrix<T> x(b.getRows(), b.getCols());
    
    size_t sys_size = R.getRows(); 
    
    for(size_t m = 0; m < b.getCols(); m++){
        
        for (size_t k = 0; k < sys_size; k++) {
            size_t i = sys_size - 1 - k; 
            T summation = static_cast<T>(0);
            for (size_t j = i + 1; j < R.getRows(); j++) { 
                summation += R(i, j) * x(j, m);
                
            }
            
            x(i, m) = (b(i, m) - summation) / R(i, i);
        }
    }
   
    return x;    
}

};