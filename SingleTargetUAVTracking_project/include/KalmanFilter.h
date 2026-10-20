/**
 * @file Kalmanfilter.h
 * @brief A state estimator based on an approach using Kalman filtering techniques. 
  
 * 
 * */ 

#pragma once
#include "MatrixCalculations.h"
#include "TrackingParameters.h"

template <typename T>
class KalmanFilter{

private:

MatrixCalculations<T> matcalc;

public:

//Update the covariance matrix P to the calibration relating to the kinematic movement phase
void updateCalibration(const UAVTracking_parameters& uavParams, Matrix<T>& P)const{
//Calibration based on kinematic movement for the UAVs
Matrix<T> IdentityMatrix_termQ = matcalc.getIdentityMatrix(3*uavParams.dimension.num_of_uavs);
T Qterm_1 = matcalc.square(uavParams.dynamical_UAV_params.max_uav_velocity*uavParams.dynamical_UAV_params.h);
T Qterm_2 = matcalc.square(uavParams.dynamical_UAV_params.max_wind_velocity*uavParams.dynamical_UAV_params.h);
T governed_vel_magnitude_terms = Qterm_1 + Qterm_2;
Matrix<T> Q = IdentityMatrix_termQ.scalarMatrix(governed_vel_magnitude_terms);

for(size_t i = 0;i < P.getRows();i++){
   for(size_t j = 0;j < P.getRows();j++){
      P(i,j) += Q(i,j);
   }
}
}

//Solve a square system of equations to achieve the Kalman Gain K_k
Matrix<T> KalmanFiltering(const UAVTracking_parameters& uavParams,const Matrix<T>& J, const Matrix<T>& P)const{

Matrix<T> Mat_term = P*J.Transpose();
//Regularization matrix to prevent ill-posed behavior (Tikhonov Regularization) for solving the system of equations below
Matrix<T> Rnoise(uavParams.dimension.num_of_landmarks*uavParams.dimension.num_of_uavs,uavParams.dimension.num_of_landmarks*uavParams.dimension.num_of_uavs);
for(size_t i = 0;i < Rnoise.getRows();i++){
   Rnoise(i,i) = uavParams.est.tau;
}
//State transition matrix
Matrix<T> S = J*Mat_term + Rnoise;
Matrix<T> rhs_term = J*P.Transpose();
Matrix<T> S_T = S.Transpose();
auto [R,b_tilde] = matcalc.QRDecomposition_GivensApproach(S_T,rhs_term,uavParams.est.eps);
Matrix<T> K = matcalc.BackwardSubstitution(R,b_tilde);

return K.Transpose();
}

//Estimate the projected step solution in order to update the UAV positions
Matrix<T> estimateState(const Matrix<T>& J, const Matrix<T>& r, Matrix<T>& P, const Matrix<T>& K)const{

Matrix<T> IdentityMatrixTerm = matcalc.getIdentityMatrix(J.getCols());
Matrix<T> proj_sol = K*r;
P = (IdentityMatrixTerm - K*J)*P;
return proj_sol;

}

//Kalman Filter method
Matrix<T> KalmanFilter_Approach(const UAVTracking_parameters& uavParams, TrackingParameters& trackingParams, const Matrix<T>& J, const Matrix<T>& r)const{

updateCalibration(uavParams,trackingParams.P);
Matrix<T> K = KalmanFiltering(uavParams,J,trackingParams.P);
Matrix<T> proj_sol = estimateState(J,r,trackingParams.P,K);
return proj_sol;
}

};
