/**
 * @file GaussNewton.h
 * @brief A state estimator based on using the Gauss-Newton strategy.
 * 
 * 
 * */ 

#pragma once
#include "MatrixCalculations.h"
#include "TrackingParameters.h"

template <typename T>
class GaussNewton {

public:

MatrixCalculations<T> matcalc;

//Gauss-Newton method
Matrix<T> GaussNewton_Approach(const UAVTracking_parameters& uavParams, TrackingParameters& trackingParams,const Matrix<T>& R, const Matrix<T>& b_tilde)const{

 //Generate Matrix Form and truncate to form an upper triangular system of equations
 auto [A,b] = matcalc.generateMatrixForm(R,b_tilde,trackingParams.lambda,uavParams.options.activate_method);
 auto [A_filtered,b_filtered] = matcalc.generateTruncatedMatrices(A,b);
 //Solve the upper triangular system of equations
 Matrix<T> dir_vec = matcalc.BackwardSubstitution(A_filtered,b_filtered);

return dir_vec;
}

};
