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

//Square, dot product, L2 measure and getting maximum of a vector functions

T maximal_vector_element(const Matrix<T>& vec)const{

T max_val = 0.0;

for(size_t i = 0;i < vec.getRows();i++){
   max_val = (max_val > vec(i,0)) ? max_val:vec(i,0); 
}

return max_val;   
}

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
auto GenerateMatrixData(const Matrix<T>& mat, const Matrix<T>& dist_truth,const Matrix<T>& dist_meas,const Matrix<T>& vec_meas)const{
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

//Reconstruct A and b if Levenberg-Marquerdt method is applied to the least square problem, otherwise keep the Jacobian and residual
auto generateMatrixForm(const Matrix<T>& A,const Matrix<T>& b, const T& lambda, const bool& activate_LM_method)const{
if(activate_LM_method){
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
size_t num_of_rotations = 0;

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
   num_of_rotations++;
   }
   }
}

return std::make_tuple(R,b_tilde,num_of_rotations);
}

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

//A term based on the Trust Region logic for the Levenberg-Marquerdt method

T GainRatio(const Matrix<T>& mat, const Matrix<T>& R, const Matrix<T>& b_tilde, const Matrix<T>& x_guess,const Matrix<T>& x_dir_vec, const T& lambda)const{

Matrix<T> x_trial = x_guess + x_dir_vec;

T nom = 0.5*(square(L2Norm(distanceMetric(mat,x_guess))) - square(L2Norm(distanceMetric(mat,x_trial))));
T denom_term1 = 0.5*(square(L2Norm(b_tilde)) - square(L2Norm(R*x_dir_vec - b_tilde)));
T denom_term2 = 0.5*lambda*square(L2Norm(x_dir_vec));

T denom = denom_term1 + denom_term2; 
T rho = nom/denom;

return rho;
}


//Help function that constructs filtered versions of any matrix A and vector b, to form an upper triangular structure. It is here the regularization
//for the matrix R happen after the truncation
auto generateTruncatedMatrices(const Matrix<T> & A, const Matrix<T>& b, const T& lambda, const bool& activate_LM_method)const {
size_t truncated_rows = std::min(A.getRows(),A.getCols());
Matrix<T> A_trunc(truncated_rows,A.getCols());
Matrix<T> b_trunc(truncated_rows,1);


for(size_t i = 0;i < A_trunc.getRows();i++){
   for(size_t j = 0;j < A_trunc.getCols();j++){
      A_trunc(i,j) = A(i,j);
   }
   b_trunc(i,0) = b(i,0);
}

for(size_t i = 0;i < A_trunc.getRows();i++){
   A_trunc(i,i) += lambda;
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

//Solver for the entire least square procedure: it is tailor-made for either using G-N or L-M
auto LeastSquareSolver_QRDecomposition(const Matrix<T>& mat,const Matrix<T>& x_truth,Matrix<T>& x_guess,
    const T& eps, const T& tau, T& lam, T& lambda_tuning_factor, const bool& activate_LM_method, const size_t& iter)const {
//Calculate distances relative to the rows of mat
//Gauss-Newton
T dir_vec_normval = 0.0;
T res_normval = 0.0;
size_t total_number_of_rotations = 0;
Matrix<T> trial_vec(3,1);
bool hasConverged = false; 
if(!activate_LM_method){
  Matrix<T> dist_truth = distanceMetric(mat,x_truth);
  Matrix<T> dist_guess = distanceMetric(mat,x_guess); 
  auto [J,r] = GenerateMatrixData(mat,dist_truth,dist_guess,x_guess); 
  auto [A,b] = generateMatrixForm(J,r,lam,activate_LM_method);
  if(iter == 0){
    lam = initializeLambda(A,tau); 
  }
  //Perform QR-decomposition of A
  auto [R,b_tilde,num_of_rotations] = QRDecomposition_GivensApproach(A,b,eps);
  auto [R_filtered,btilde_filtered] = generateTruncatedMatrices(R,b_tilde,lam,activate_LM_method);
  //Solve the upper triangular system of equations
  Matrix<T> dir_vec = BackwardSubstitution(R_filtered,btilde_filtered);
  dir_vec_normval = L2Norm(dir_vec);
  trial_vec = dir_vec + x_guess;
  x_guess = trial_vec;
  res_normval = L2Norm(distanceMetric(mat,trial_vec));
  total_number_of_rotations += num_of_rotations;
}
else{
   T tuning_factor = lambda_tuning_factor;
   bool snapshotContinued = true;
   Matrix<T> dist_truth = distanceMetric(mat,x_truth);
   Matrix<T> dist_guess = distanceMetric(mat,x_guess);
   auto [J,r] = GenerateMatrixData(mat,dist_truth,dist_guess,x_guess);
   auto [R,b_tilde,num_of_rotations] = QRDecomposition_GivensApproach(J,r,eps);
   res_normval = L2Norm(r);
   total_number_of_rotations += num_of_rotations;
    if(iter == 0){
      lam = initializeLambda(J,tau); 
    }
   while(snapshotContinued){
       auto [A,b] = generateMatrixForm(R,b_tilde,lam,activate_LM_method);      
       auto [R_aug,b_tilde_aug,num_of_rotations] = QRDecomposition_GivensApproach(A,b,eps);
       auto [R_filtered,btilde_filtered] = generateTruncatedMatrices(R_aug,b_tilde_aug,lam,activate_LM_method);
       Matrix<T> dir_vec = BackwardSubstitution(R_filtered,btilde_filtered);
       trial_vec = dir_vec + x_guess;
       T dist_trial_normval = L2Norm(trial_vec);
       T rho = GainRatio(mat,R_filtered,btilde_filtered,x_guess,dir_vec,lam);
       std::cout<< "\nLambda: " << lam << "\n";
       std::cout<< "\nRho: " << rho << "\n";
       if(rho > 0){
         lam *= std::max(1.0/3.0,1.0 - std::pow((2.0*rho - 1.0),3));
         tuning_factor = lambda_tuning_factor;
         x_guess = trial_vec;
         dir_vec_normval = L2Norm(dir_vec);
         res_normval = dist_trial_normval;
         total_number_of_rotations += num_of_rotations;
         snapshotContinued = false;
       }
       else{
          lam *= tuning_factor;
          tuning_factor = lambda_tuning_factor*tuning_factor;
       }


   }
} 
return std::make_tuple(x_guess,dir_vec_normval,res_normval,total_number_of_rotations,lam);
}

};