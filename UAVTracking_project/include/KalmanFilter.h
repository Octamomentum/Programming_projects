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

//Generating the covariance matrix that measures uncertainty in radar measurements
Matrix<T> generateR(const UAVTracking_parameters& uavParams)const{

size_t m = uavParams.dimension.num_of_landmarks*uavParams.dimension.num_of_uavs;

//In the ideal case, R is gamma*I where gamma is a small constant
if(!uavParams.options.applyWGN){
  return matcalc.getIdentityMatrix(m).scalarMatrix(uavParams.est.gamma);
}
//When WGN is added, R is a covariance matrix consisting of its diagonal elements as the variance from the generated WGN to the true distances
return matcalc.getIdentityMatrix(m).scalarMatrix(uavParams.variance.variance_distances);
}

//Update the covariance matrix P based on the calibration relating to the kinematic UAV movement phase
void updateCalibration(const UAVTracking_parameters& uavParams, Matrix<T>& P)const{
//Covariance matrix Q describing the uncertainty in the kinematic UAV movement
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
Matrix<T> KalmanFiltering(const UAVTracking_parameters& uavParams,const Matrix<T>& P, const Matrix<T>& J)const{

Matrix<T> Mat_term = P*J.Transpose();
//Covariance matrix measuring uncertainty in the radar measurements
Matrix<T> R_noise = generateR(uavParams);
//State transition matrix
Matrix<T> S = J*Mat_term + R_noise;
Matrix<T> rhs_term = J*P.Transpose();
Matrix<T> S_T = S.Transpose();
auto [R,b_tilde] = matcalc.QRDecomposition_GivensApproach(S_T,rhs_term,uavParams.est.eps);
Matrix<T> K = matcalc.BackwardSubstitution(R,b_tilde);

return K.Transpose();
}

//Estimate the projected solution step in order to update the UAV positions
Matrix<T> estimateState(const Matrix<T>& J, const Matrix<T>& r, Matrix<T>& P, const Matrix<T>& K)const{

Matrix<T> IdentityMatrixTerm = matcalc.getIdentityMatrix(J.getCols());
Matrix<T> proj_sol = K*r;
P = (IdentityMatrixTerm - K*J)*P;
return proj_sol;

}

//Kalman Filter method
Matrix<T> KalmanFilter_Approach(const UAVTracking_parameters& uavParams, TrackingParameters& trackingParams, const Matrix<T>& J, const Matrix<T>& r)const{

updateCalibration(uavParams,trackingParams.P);
Matrix<T> K = KalmanFiltering(uavParams,trackingParams.P,J);
Matrix<T> proj_sol = estimateState(J,r,trackingParams.P,K);
return proj_sol;
}

};
