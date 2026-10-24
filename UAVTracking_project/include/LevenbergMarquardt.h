/**
 * @file LevenbergMarquardt.h
 * @brief A state estimator based on using the Levenberg-Marquardt strategy.
  
 * 
 * */ 

#pragma once
#include "MatrixCalculations.h"
#include "TrackingParameters.h"

template <typename T>
class LevenbergMarquardt{

private:

MatrixCalculations<T> matcalc;

public:

//A term based on the Trust Region logic for the Levenberg-Marquardt method
auto GainRatio(TrackingParameters& trackingParams, const Matrix<T>& R, const Matrix<T>& b_tilde,const Matrix<T>& x_dir_vec)const{
//Transform vectorized step solution to appropriate matrix form
Matrix<T> transformedSolution = matcalc.transformVectorToMatrix(x_dir_vec,trackingParams.UAVGuess.getRows());
Matrix<T> x_trial = trackingParams.UAVGuess + transformedSolution;
T nom = 0.0;
T rho = 0.0;
//Actual cost function value
for(size_t i = 0;i < transformedSolution.getRows();i++){
   Matrix<T> row_vec1 = trackingParams.UAVGuess.getRow(i);
   Matrix<T> row_vec2 = x_trial.getRow(i);
   T nom_1 = matcalc.square(matcalc.L2Norm(matcalc.distanceMetric(trackingParams.mat,row_vec1)));
   T nom_2 = matcalc.square(matcalc.L2Norm(matcalc.distanceMetric(trackingParams.mat,row_vec2)));
   nom += 0.5*(nom_1 - nom_2); 
}
//Expected cost function value via QR-decomposition
T denom_term1 = 0.5*(matcalc.square(matcalc.L2Norm(b_tilde)) - matcalc.square(matcalc.L2Norm(R*x_dir_vec - b_tilde)));
T denom_term2 = 0.5*trackingParams.lambda*matcalc.square(matcalc.L2Norm(x_dir_vec));

T denom = denom_term1 - denom_term2;
//Preventing NaN-situations
if(std::abs(denom) > 1e-2){
   rho = nom/denom;

}

return rho;
}

//Augment J and r as the Levenberg-Marquardt method is applied to the UAV Tracking problem solver
auto generateMatrixForm(const Matrix<T>& A,const Matrix<T>& b, const T& lambda)const{

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

//Levenberg-Marquardt method
Matrix<T> LevenbergMarquardt_Approach(const UAVTracking_parameters& uavParams, TrackingParameters& trackingParams, const Matrix<T>& R,
   const Matrix<T>& b_tilde)const{

Matrix<T> dir_vec(R.getRows(),1);
 T lambda_amplifier = uavParams.est.lambda_tuning_factor; 
   bool snapshotContinued = true;
   //Perform a number of trials to estimate the Gain Ratio rho
       while(snapshotContinued){
      //Generate Matrix form
       auto [A,b] = generateMatrixForm(R,b_tilde,trackingParams.lambda);
       //QR-decompose A based on the L-M strategy      
       auto [A_aug,b_aug] = matcalc.QRDecomposition_GivensApproach(A,b,uavParams.est.eps);
       auto [R_filtered,btilde_filtered] = matcalc.generateTruncatedMatrices(A_aug,b_aug);
       //Solve upper-triangular system of equations
       dir_vec = matcalc.BackwardSubstitution(R_filtered,btilde_filtered);
       //Calculate Gain Ratio for making a trial decision
       T rho = GainRatio(trackingParams,R_filtered,btilde_filtered,dir_vec);
       std::cout<< "\nLambda: " << trackingParams.lambda << "\n";
       std::cout<< "\nRho: " << rho << "\n";
       if(std::abs(rho) < uavParams.est.rho_threshold || trackingParams.lambda == 0.0){
         break;
       }
       if(rho > 0){
         //Design choice for the Trust Region approach when lambda decreases
         trackingParams.lambda *= std::max(uavParams.est.lambda_threshold,1.0 - std::pow((2.0*rho - 1.0),3));
         lambda_amplifier =  uavParams.est.lambda_tuning_factor;
         snapshotContinued = false;
       }
       else{
          //Increase lambda when rho is non-positive
          trackingParams.lambda *= lambda_amplifier;
          lambda_amplifier *= uavParams.est.lambda_tuning_factor;
       }
}
return dir_vec;
}

};
