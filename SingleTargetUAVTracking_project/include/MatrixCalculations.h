/**
 * @file MatrixCalculations.h
 * @brief Performs calculations using matrix structure.
 * 
 * Implements a Gauss-Newton strategy that constructs a Jacobian matrix J and a residual vector r to perform data assimilation later 
 * in the UAVTracking file. In the strategy, it calculates residual distances based on the Euclidean measure row-wise between 
 * an overdetermined matrix and a vector to form A and b, which later is used for solving the least squares problem using thin QR decomposition of A.
 * The algorithm for the thin QR decomposition method follows the logic of generating Givens rotations. 
 * 
 * Two solvers are implemented in this project: the classic Gauss-Newton and Levenberg-Marquardt. For a future update, methods like Kalman filter and BFGS
 * will be implemented for this project.
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

UAVTracking_parameters uavParams;
GenerateWGNSource<T> generate_wgn;

public:



struct LeastSquareOutput{

T dir_vec_normval = 0.0;
T res_normval = 0.0;
};


struct TrackingParams{

};

//Help method for initializing lambda, which retrieves the largest value from a vector

T maximal_vector_element(const Matrix<T>& vec)const{
T max_val = 0.0;
for(size_t i = 0;i < vec.getRows();i++){
   max_val = (max_val > vec(i,0)) ? max_val:vec(i,0); 
}
return max_val;  
}

//Square method

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

//Method that constructs a Jacobian matrix and residual vector for data assimilation
auto GenerateMatrixData(const Matrix<T>& mat, const Matrix<T>& dist_truth,const Matrix<T>& dist_meas,const Matrix<T>& vec_meas)const{
Matrix<T> J(mat.getRows(),3);
Matrix<T> r(mat.getRows(),1);
for(size_t i = 0;i < mat.getRows();i++){
   for(size_t j = 0;j < 3;j++){
      J(i,j) = (vec_meas(j,0)-mat(i,j))/dist_meas(i,0);
   }
}
for(size_t i = 0;i < mat.getRows();i++){
   r(i,0) = dist_truth(i,0) - dist_meas(i,0); 
}
return std::make_tuple(J,r);
}

//Reconstruct J and r if Levenberg-Marquardt method is applied to the least squares problem, otherwise keep the Jacobian and residual
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

//Method that retrieves the first lambda by default w.r.t. the parameter tau, treated as an initialized regularization term
T initializeLambda(const Matrix<T>& A, const T& tau)const{

Matrix<T> potential_lambda(A.getCols(),1);
T lambda = 0.0;

for(size_t i = 0;i < A.getCols();i++){
   Matrix<T> col_vec = A.getColumn(i);
   T summation = 0.0;
   for(size_t j = 0;j < col_vec.getRows();j++){
      summation += square(col_vec(j,0));
   }
   potential_lambda(i,0) = summation;
}

lambda = tau*maximal_vector_element(potential_lambda);

return lambda;   
}

//A term based on the Trust Region logic for the Levenberg-Marquardt method

auto GainRatio(const Matrix<T>& mat, const Matrix<T>& R, const Matrix<T>& b_tilde,
    const Matrix<T>& x_guess,const Matrix<T>& x_dir_vec, const T& lambda)const{

Matrix<T> x_trial = x_guess + x_dir_vec;
bool isDenominatorIdenticallyZero = false;
//Expected cost function value
T nom = 0.5*(square(L2Norm(distanceMetric(mat,x_guess))) - square(L2Norm(distanceMetric(mat,x_trial))));
//Actual cost function values via QR-decomposition
T denom_term1 = 0.5*(square(L2Norm(b_tilde)) - square(L2Norm(R*x_dir_vec - b_tilde)));
T denom_term2 = 0.5*lambda*square(L2Norm(x_dir_vec));

T denom = denom_term1 + denom_term2;
//Preventing NaN-situations
if(denom == 0.0){
  isDenominatorIdenticallyZero = true;
}
T rho = nom/denom;

return std::make_tuple(rho,isDenominatorIdenticallyZero);
}


//Help function that constructs filtered versions of any matrix A and vector b, to form an upper triangular structure.
 //It is here the Tikhonov regularization for the matrix R happen after the truncation
auto generateTruncatedMatrices(const UAVTracking_parameters& uavParams,const Matrix<T> & A, const Matrix<T>& b, const T& lambda)const {
size_t truncated_rows = std::min(A.getRows(),A.getCols());
Matrix<T> A_trunc(truncated_rows,A.getCols());
Matrix<T> b_trunc(truncated_rows,1);


for(size_t i = 0;i < A_trunc.getRows();i++){
   for(size_t j = 0;j < A_trunc.getCols();j++){
      A_trunc(i,j) = A(i,j);
   }
   b_trunc(i,0) = b(i,0);
}
//Apply Tikuhonov-regularization only when WGN is added for the G-N case
if(uavParams.options.activate_method == "G-N" && uavParams.options.applyWGN){
  for(size_t i = 0;i < A_trunc.getRows();i++){
     A_trunc(i,i) += lambda;
  } 
} 

return std::make_tuple(A_trunc, b_trunc);
}

//Solves an upper-triangular system of equations
Matrix<T> BackwardSubstitution(const Matrix<T>& R, const Matrix<T>& b) const {
    Matrix<T> x(b.getRows(),1);
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


//Identity matrix for a later update: BFGS implementation
Matrix<T> getIdentityMatrix(const size_t& m)const{

Matrix<T> IdentityMatrix(m,m);

for(size_t i = 0; i < m;i++){
   IdentityMatrix(i,i) = 1.0;
}

return IdentityMatrix;
}

//The least squares solver: it either uses Gauss-Newton or Levenberg-Marquardt for each iteration
auto LeastSquares_Solver(const UAVTracking_parameters& uavParams, TrackingParameters& trackingParams, const size_t& iter,
    const Matrix<size_t>& variance_distance_seed)const{
  
  LeastSquareOutput ls_output;
  Matrix<T> trial_vec(3,1);

  //Calculate distances
  Matrix<T> dist_truth = distanceMetric(trackingParams.mat,trackingParams.x_truth);
  Matrix<T> dist_guess = distanceMetric(trackingParams.mat,trackingParams.x_guess);
  //Apply WGN if the option is possible
  if(uavParams.options.applyWGN){
    generate_wgn.AddingWGNToTrueDistances(dist_truth,variance_distance_seed(iter,0),uavParams.variance.variance_distances);
  }
  //Generate Jacobian J and residual vector r
  auto [J,r] = GenerateMatrixData(trackingParams.mat,dist_truth,dist_guess,trackingParams.x_guess); 
  //Perform QR-decomposition of J
  auto [R,b_tilde] = QRDecomposition_GivensApproach(J,r,uavParams.least_square.eps); 
  ls_output.res_normval = L2Norm(r);
   //Initialize damping
  if(iter == 0){ 
    trackingParams.lambda = initializeLambda(R,uavParams.least_square.tau); 
  }
  //Calculate the L2-norm of the residuals and collect the number of rotations

  //Perform G-N if such option is chosen
  if(uavParams.options.activate_method == "G-N"){

     //Generate Matrix Form and truncate to form a upper triangular system of equations
     auto [A,b] = generateMatrixForm(R,b_tilde,trackingParams.lambda,uavParams.options.activate_method);
     auto [A_filtered,b_filtered] = generateTruncatedMatrices(uavParams,A,b,trackingParams.lambda);
     //Solve the upper triangular system of equations
     Matrix<T> dir_vec = BackwardSubstitution(A_filtered,b_filtered);
     //Calculate L2 norm for the direction vector and update estimated coordinates
     ls_output.dir_vec_normval = L2Norm(dir_vec);
     trial_vec = trackingParams.x_guess + dir_vec;
     trackingParams.x_guess = trial_vec;
     //Calculate the number of Givens rotations and the number of runs per trial

     return ls_output;
    }
   T lambda_amplifier = uavParams.least_square.lambda_tuning_factor; 
  //Perform L-M if such option is chosen
   bool snapshotContinued = true;
   
       while(snapshotContinued){
      //Generate Matrix Form
       auto [A,b] = generateMatrixForm(R,b_tilde,trackingParams.lambda,uavParams.options.activate_method);
       //QR-decompose A based on the L-M strategy      
       auto [A_aug,b_aug] = QRDecomposition_GivensApproach(A,b,uavParams.least_square.eps);
       auto [R_filtered,btilde_filtered] = generateTruncatedMatrices(uavParams,A_aug,b_aug,trackingParams.lambda);
       //Solve least squares problem
       Matrix<T> dir_vec = BackwardSubstitution(R_filtered,btilde_filtered);
       trial_vec = trackingParams.x_guess + dir_vec;
       //Calculate Gain Ratio for make a trial decision
       auto [rho,isDenominatorZero] = GainRatio(trackingParams.mat,R_filtered,btilde_filtered,trackingParams.x_guess,dir_vec,trackingParams.lambda);
       std::cout<< "\nLambda: " << trackingParams.lambda << "\n";
       std::cout<< "\nRho: " << rho << "\n";
       if(std::abs(rho) < uavParams.least_square.rho_threshold || isDenominatorZero){
         break;
       }
       if(rho > 0){
         //Design choice for the Trust Region approach when lambda decrease
         trackingParams.lambda *= std::max(uavParams.least_square.lambda_threshold,1.0 - std::pow((2.0*rho - 1.0),3));
         lambda_amplifier =  uavParams.least_square.lambda_tuning_factor;
         trackingParams.x_guess = trial_vec;
         ls_output.dir_vec_normval = L2Norm(dir_vec);
         snapshotContinued = false;
       }
       else{
          //Increase lambda when rho is non-positive
          trackingParams.lambda *= lambda_amplifier;
          lambda_amplifier *= uavParams.least_square.lambda_tuning_factor;
       }


   
}
  return ls_output;
}

};